// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal PDMA ethernet driver for the MT7510-class frame engine
 * (chip id 0x0005xxxx at 0x1fb00064) found in the ZTE ZXHN H267A hw V1.0.2.
 * This silicon has no QDMA block, so econet_eth cannot drive it.
 *
 * One TX ring, one RX ring, NAPI on the FE interrupt (irq_mode=0 polls).
 * Register layout and descriptor format follow the vendor bootloader's
 * femac.c / femac_7510.h (PDMA_BYTE_SWAP on big-endian, PN at bits 21:19).
 * Acts as the DSA conduit for the on-die MT7530 (frames pass through
 * untouched, MTK special tag included).
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/reset.h>

#define FE_GDM1_FWD_CFG		0x500
#define FE_GDM1_MAC_L		0x508
#define FE_GDM1_LEN_CFG		0x514
#define FE_GDM1_MAC_H		0x50c

#define PDMA_TX_BASE		0x800
#define PDMA_TX_MAX		0x804
#define PDMA_TX_CTX		0x808
#define PDMA_TX_DTX		0x80c
#define PDMA_RX_BASE		0x900
#define PDMA_RX_MAX		0x904
#define PDMA_RX_CALC		0x908
#define PDMA_RX_DRX		0x90c
#define PDMA_GLO_CFG		0xa04
#define PDMA_RST_IDX		0xa08
#define PDMA_DLY_INT		0xa0c
#define PDMA_INT_STATUS		0xa20
#define PDMA_INT_MASK		0xa28

#define GLO_TX_DMA_EN		BIT(0)
#define GLO_TX_DMA_BUSY		BIT(1)
#define GLO_RX_DMA_EN		BIT(2)
#define GLO_RX_DMA_BUSY		BIT(3)
#define GLO_TX_WB_DDONE		BIT(6)
#define GLO_RX_2B_OFFSET	BIT(31)

#define D2_DDONE		BIT(31)
#define D2_LS0			BIT(30)
#define D2_PLEN0(x)		(((x) & 0x3fff) << 16)
#define D2_GET_PLEN0(x)		(((x) >> 16) & 0x3fff)

#define TX_N			128
#define RX_N			128
#define RX_BUF			1920

static int fport = 1;
module_param(fport, int, 0644);
static int fport_shift = 19;
module_param(fport_shift, int, 0644);
static int rx_ls0;
module_param(rx_ls0, int, 0644);
static int irq_mode = 0x10001;	/* RX_DONE_INT0 | TX_DONE_INT0; 0 = timer polling */
module_param(irq_mode, int, 0644);
/* vendor macSetMACCR: jumbo len 12, strip CRC, everything to the CPU (PDMA) */
static uint fwd_cfg = 0xc0010000;
module_param(fwd_cfg, uint, 0644);
static int dbg;
module_param(dbg, int, 0644);
static int do_reset = 1;
module_param(do_reset, int, 0644);
/* extra PDMA_GLO_CFG bits for experiments (bit 29 = BYTE_SWAP) */
static int glo_or = 0x20000000;
module_param(glo_or, int, 0644);

/* vendor femac.c: on a big-endian CPU set PDMA_BYTE_SWAP (bit 29) and use native-order descriptors */
struct pdma_desc {
	__le32 d1, d2, d3, d4;
};
static int le_desc;
module_param(le_desc, int, 0644);
#define DR(x)	(le_desc ? le32_to_cpu(READ_ONCE(x)) : (__force u32)READ_ONCE(x))
#define DW(x, v)	WRITE_ONCE(x, le_desc ? cpu_to_le32(v) : (__force __le32)(u32)(v))

struct mt7510_eth {
	void __iomem *base;
	struct device *dev;
	struct net_device *ndev;
	struct napi_struct napi;
	struct delayed_work poll_work;
	struct pdma_desc *tx, *rx;
	dma_addr_t tx_phys, rx_phys;
	struct sk_buff *tx_skb[TX_N], *rx_skb[RX_N];
	dma_addr_t tx_map[TX_N], rx_map[RX_N];
	unsigned int tx_len[TX_N];
	unsigned int tx_head, tx_tail, rx_idx;
	atomic_t tx_free;
	int irq;
	unsigned long dbg_next;
	struct reset_control *rst;
	bool running;
};

static inline u32 rr(struct mt7510_eth *p, u32 o) { return readl(p->base + o); }
static inline void wr(struct mt7510_eth *p, u32 o, u32 v) { writel(v, p->base + o); }

static u32 rx_d2_init(void)
{
	return rx_ls0 ? D2_LS0 : (D2_LS0 | D2_PLEN0(RX_BUF - 2));
}

static struct sk_buff *mt_rx_alloc(struct mt7510_eth *p, dma_addr_t *map)
{
	struct sk_buff *skb = netdev_alloc_skb(p->ndev, RX_BUF);

	if (!skb)
		return NULL;
	*map = dma_map_single(p->dev, skb->data, RX_BUF, DMA_FROM_DEVICE);
	if (dma_mapping_error(p->dev, *map)) {
		dev_kfree_skb_any(skb);
		return NULL;
	}
	return skb;
}

static void mt_tx_reclaim(struct mt7510_eth *p)
{
	struct net_device *ndev = p->ndev;
	unsigned int t = p->tx_tail;
	int n = 0;

	while (t != READ_ONCE(p->tx_head) && (DR(p->tx[t].d2) & D2_DDONE)) {
		dma_unmap_single(p->dev, p->tx_map[t], p->tx_len[t], DMA_TO_DEVICE);
		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += p->tx_len[t];
		dev_consume_skb_any(p->tx_skb[t]);
		p->tx_skb[t] = NULL;
		t = (t + 1) % TX_N;
		n++;
	}
	p->tx_tail = t;
	if (n) {
		atomic_add(n, &p->tx_free);
		if (netif_queue_stopped(ndev) && atomic_read(&p->tx_free) > TX_N / 4)
			netif_wake_queue(ndev);
	}
}

static int mt_rx(struct mt7510_eth *p, int budget)
{
	struct net_device *ndev = p->ndev;
	int done = 0;

	while (done < budget) {
		unsigned int i = p->rx_idx;
		struct pdma_desc *d = &p->rx[i];
		u32 d2 = DR(d->d2);
		struct sk_buff *skb, *nskb;
		dma_addr_t nmap;
		unsigned int len;

		if (!(d2 & D2_DDONE))
			break;

		len = D2_GET_PLEN0(d2);
		nskb = mt_rx_alloc(p, &nmap);
		if (!nskb || len < ETH_HLEN || len > RX_BUF) {
			ndev->stats.rx_dropped++;
			if (nskb) {
				dma_unmap_single(p->dev, nmap, RX_BUF, DMA_FROM_DEVICE);
				dev_kfree_skb_any(nskb);
			}
		} else {
			skb = p->rx_skb[i];
			dma_unmap_single(p->dev, p->rx_map[i], RX_BUF, DMA_FROM_DEVICE);
			skb_reserve(skb, 2);
			skb_put(skb, len);
			skb->protocol = eth_type_trans(skb, ndev);
			ndev->stats.rx_packets++;
			ndev->stats.rx_bytes += len;
			napi_gro_receive(&p->napi, skb);
			p->rx_skb[i] = nskb;
			p->rx_map[i] = nmap;
			DW(d->d1, nmap);
		}
		DW(d->d3, 0);
		DW(d->d4, 0);
		wmb();
		DW(d->d2, rx_d2_init());
		wmb();
		wr(p, PDMA_RX_CALC, i);
		p->rx_idx = (i + 1) % RX_N;
		done++;
	}
	return done;
}

static int mt_poll(struct napi_struct *napi, int budget)
{
	struct mt7510_eth *p = container_of(napi, struct mt7510_eth, napi);
	int done;

	mt_tx_reclaim(p);
	done = mt_rx(p, budget);
	if (done < budget && napi_complete_done(napi, done)) {
		if (irq_mode && p->irq > 0)
			wr(p, PDMA_INT_MASK, irq_mode);
		else if (p->running)
			schedule_delayed_work(&p->poll_work, 1);
	}
	return done;
}

static void mt_poll_work(struct work_struct *w)
{
	struct mt7510_eth *p = container_of(to_delayed_work(w), struct mt7510_eth, poll_work);

	if (dbg && p->running && time_after(jiffies, p->dbg_next)) {
		unsigned int r = p->rx_idx, h = rr(p, PDMA_RX_DRX) % RX_N, t = p->tx_tail;

		p->dbg_next = jiffies + 2 * HZ;
		netdev_info(p->ndev, "dbg rx_idx=%u drx=%u calc=%u rx[idx]=%08x/%08x/%08x rx[0]=%08x/%08x rx[drx-1]=%08x/%08x | tx tail=%u head=%u ctx=%u dtx=%u tx[tail]=%08x/%08x/%08x int=%08x pse=%08x iq=%08x oq=%08x\n",
			    r, h, rr(p, PDMA_RX_CALC), DR(p->rx[r].d1), DR(p->rx[r].d2), DR(p->rx[r].d4),
			    DR(p->rx[0].d1), DR(p->rx[0].d2), DR(p->rx[(h + RX_N - 1) % RX_N].d2), DR(p->rx[(h + RX_N - 1) % RX_N].d4),
			    t, p->tx_head, rr(p, PDMA_TX_CTX), rr(p, PDMA_TX_DTX), DR(p->tx[t].d1), DR(p->tx[t].d2), DR(p->tx[t].d4),
			    rr(p, PDMA_INT_STATUS), rr(p, 0x100), rr(p, 0x110), rr(p, 0x118));
	}

	if (p->running)
		napi_schedule(&p->napi);
}

static irqreturn_t mt_irq(int irq, void *data)
{
	struct mt7510_eth *p = data;
	u32 st = rr(p, PDMA_INT_STATUS);

	if (!st)
		return IRQ_NONE;
	wr(p, PDMA_INT_STATUS, st);
	wr(p, PDMA_INT_MASK, 0);
	napi_schedule(&p->napi);
	return IRQ_HANDLED;
}

static netdev_tx_t mt_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct mt7510_eth *p = netdev_priv(ndev);
	unsigned int i = p->tx_head, len;
	struct pdma_desc *d = &p->tx[i];
	dma_addr_t map;

	if (skb_put_padto(skb, ETH_ZLEN))
		return NETDEV_TX_OK;
	if (skb_linearize(skb)) {
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
	if (atomic_read(&p->tx_free) < 1) {
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}
	len = skb->len;
	map = dma_map_single(p->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(p->dev, map)) {
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
	p->tx_skb[i] = skb;
	p->tx_map[i] = map;
	p->tx_len[i] = len;
	DW(d->d1, map);
	DW(d->d3, 0);
	DW(d->d4, (u32)fport << fport_shift);
	wmb();
	DW(d->d2, D2_PLEN0(len) | D2_LS0);
	wmb();
	WRITE_ONCE(p->tx_head, (i + 1) % TX_N);
	wr(p, PDMA_TX_CTX, p->tx_head);
	if (atomic_dec_return(&p->tx_free) < 1)
		netif_stop_queue(ndev);
	return NETDEV_TX_OK;
}

static void mt_dma_stop(struct mt7510_eth *p)
{
	int i;

	wr(p, PDMA_INT_MASK, 0);
	wr(p, PDMA_GLO_CFG, rr(p, PDMA_GLO_CFG) & ~(GLO_TX_DMA_EN | GLO_RX_DMA_EN));
	for (i = 0; i < 100; i++) {
		if (!(rr(p, PDMA_GLO_CFG) & (GLO_TX_DMA_BUSY | GLO_RX_DMA_BUSY)))
			break;
		msleep(1);
	}
}

static void mt_write_mac(struct mt7510_eth *p)
{
	const u8 *m = p->ndev->dev_addr;

	wr(p, FE_GDM1_MAC_L, (m[2] << 24) | (m[3] << 16) | (m[4] << 8) | m[5]);
	wr(p, FE_GDM1_MAC_H, (m[0] << 8) | m[1]);
}

static int mt_open(struct net_device *ndev)
{
	struct mt7510_eth *p = netdev_priv(ndev);
	int i;

	mt_dma_stop(p);
	if (p->rst) {
		/* vendor: 'FE_RST' bit 1 of RSTCTRL2 means PDMA reset */
		reset_control_assert(p->rst);
		usleep_range(1000, 2000);
		reset_control_deassert(p->rst);
		usleep_range(1000, 2000);
	}
	wr(p, 0x004, 1);	/* FE_RST_GLO: PSE reset (self-clearing) */
	usleep_range(1000, 2000);
	wr(p, 0x004, 0);
	wr(p, FE_GDM1_FWD_CFG, fwd_cfg);
	/* GDMA1 frame length limits: max in [31:16], min in [15:0]. The reset
	 * value (1514) drops full-size frames once the 4-byte DSA tag is added. */
	wr(p, FE_GDM1_LEN_CFG, (1536 << 16) | ETH_ZLEN);
	mt_write_mac(p);
	for (i = 0; i < TX_N; i++) {
		DW(p->tx[i].d1, 0);
		DW(p->tx[i].d3, 0);
		DW(p->tx[i].d4, (u32)fport << fport_shift);
		DW(p->tx[i].d2, D2_DDONE | D2_LS0);
	}
	for (i = 0; i < RX_N; i++) {
		if (!p->rx_skb[i]) {
			p->rx_skb[i] = mt_rx_alloc(p, &p->rx_map[i]);
			if (!p->rx_skb[i])
				return -ENOMEM;
		}
		DW(p->rx[i].d1, p->rx_map[i]);
		DW(p->rx[i].d3, 0);
		DW(p->rx[i].d4, 0);
		DW(p->rx[i].d2, rx_d2_init());
	}
	wmb();
	p->tx_head = 0;
	p->tx_tail = 0;
	p->rx_idx = 0;
	atomic_set(&p->tx_free, TX_N - 1);

	wr(p, PDMA_TX_BASE, p->tx_phys);
	wr(p, PDMA_TX_MAX, TX_N);
	wr(p, PDMA_TX_CTX, 0);
	wr(p, PDMA_RX_BASE, p->rx_phys);
	wr(p, PDMA_RX_MAX, RX_N);
	wr(p, PDMA_RX_CALC, RX_N - 1);
	wr(p, PDMA_RST_IDX, BIT(0));
	wr(p, PDMA_RST_IDX, BIT(16));
	wr(p, PDMA_DLY_INT, 0);
	wr(p, PDMA_INT_STATUS, 0xffffffff);

	wr(p, PDMA_GLO_CFG, rr(p, PDMA_GLO_CFG) | GLO_RX_2B_OFFSET |
	   GLO_TX_WB_DDONE | glo_or | GLO_TX_DMA_EN | GLO_RX_DMA_EN);

	p->running = true;
	napi_enable(&p->napi);
	netif_carrier_on(ndev);
	netif_start_queue(ndev);
	if (irq_mode && p->irq > 0)
		wr(p, PDMA_INT_MASK, irq_mode);
	else
		schedule_delayed_work(&p->poll_work, 1);
	netdev_info(ndev, "up: glo=%08x txbase=%08x rxbase=%08x fwd=%08x\n",
		    rr(p, PDMA_GLO_CFG), rr(p, PDMA_TX_BASE), rr(p, PDMA_RX_BASE),
		    rr(p, FE_GDM1_FWD_CFG));
	return 0;
}

static int mt_stop(struct net_device *ndev)
{
	struct mt7510_eth *p = netdev_priv(ndev);
	unsigned int t;

	p->running = false;
	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	mt_dma_stop(p);
	cancel_delayed_work_sync(&p->poll_work);
	napi_disable(&p->napi);
	for (t = 0; t < TX_N; t++) {
		if (p->tx_skb[t]) {
			dma_unmap_single(p->dev, p->tx_map[t], p->tx_len[t], DMA_TO_DEVICE);
			dev_kfree_skb_any(p->tx_skb[t]);
			p->tx_skb[t] = NULL;
		}
	}
	return 0;
}

static int mt_set_mac(struct net_device *ndev, void *addr)
{
	struct mt7510_eth *p = netdev_priv(ndev);
	int err = eth_mac_addr(ndev, addr);
	const u8 *m = ndev->dev_addr;

	if (err)
		return err;
	wr(p, FE_GDM1_MAC_L, (m[2] << 24) | (m[3] << 16) | (m[4] << 8) | m[5]);
	wr(p, FE_GDM1_MAC_H, (m[0] << 8) | m[1]);
	return 0;
}

static const struct net_device_ops mt_ops = {
	.ndo_open		= mt_open,
	.ndo_stop		= mt_stop,
	.ndo_start_xmit		= mt_xmit,
	.ndo_set_mac_address	= mt_set_mac,
	.ndo_validate_addr	= eth_validate_addr,
};

static int mt_probe(struct platform_device *pdev)
{
	struct device_node *np, *mac_np = NULL;
	struct net_device *ndev;
	struct mt7510_eth *p;
	const u8 *m;
	int err;

	ndev = devm_alloc_etherdev(&pdev->dev, sizeof(*p));
	if (!ndev)
		return -ENOMEM;
	p = netdev_priv(ndev);
	p->ndev = ndev;
	p->dev = &pdev->dev;
	platform_set_drvdata(pdev, p);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);

	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (err)
		return err;

	for_each_available_child_of_node(pdev->dev.of_node, np) {
		if (of_device_is_compatible(np, "econet,eth-mac")) {
			mac_np = np;
			break;
		}
	}
	if (mac_np) {
		ndev->dev.of_node = mac_np;
		if (of_get_ethdev_address(mac_np, ndev))
			eth_hw_addr_random(ndev);
	} else {
		eth_hw_addr_random(ndev);
	}

	p->tx = dmam_alloc_coherent(&pdev->dev, TX_N * sizeof(*p->tx), &p->tx_phys, GFP_KERNEL);
	p->rx = dmam_alloc_coherent(&pdev->dev, RX_N * sizeof(*p->rx), &p->rx_phys, GFP_KERNEL);
	if (!p->tx || !p->rx)
		return -ENOMEM;

	if (do_reset) {
		p->rst = devm_reset_control_get_optional_exclusive(&pdev->dev, "qdma0");
		if (IS_ERR(p->rst)) {
			dev_warn(&pdev->dev, "no pdma reset: %pe\n", p->rst);
			p->rst = NULL;
		}
	}

	INIT_DELAYED_WORK(&p->poll_work, mt_poll_work);
	ndev->netdev_ops = &mt_ops;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = RX_BUF - ETH_HLEN - 2 * VLAN_HLEN - ETH_FCS_LEN - 8;
	netif_napi_add(ndev, &p->napi, mt_poll);

	p->irq = platform_get_irq_optional(pdev, 0);
	if (irq_mode && p->irq > 0) {
		err = devm_request_irq(&pdev->dev, p->irq, mt_irq, 0, "mt7510_eth", p);
		if (err)
			return err;
	}

	m = ndev->dev_addr;
	wr(p, FE_GDM1_MAC_L, (m[2] << 24) | (m[3] << 16) | (m[4] << 8) | m[5]);
	wr(p, FE_GDM1_MAC_H, (m[0] << 8) | m[1]);

	err = register_netdev(ndev);
	if (err)
		return err;
	dev_info(&pdev->dev, "MT7510 PDMA eth registered as %s, %pM (fport=%d<<%d rx_ls0=%d irq_mode=%x)\n",
		 ndev->name, ndev->dev_addr, fport, fport_shift, rx_ls0, irq_mode);
	return 0;
}

static void mt_remove(struct platform_device *pdev)
{
	struct mt7510_eth *p = platform_get_drvdata(pdev);
	int i;

	unregister_netdev(p->ndev);
	netif_napi_del(&p->napi);
	for (i = 0; i < RX_N; i++) {
		if (p->rx_skb[i]) {
			dma_unmap_single(p->dev, p->rx_map[i], RX_BUF, DMA_FROM_DEVICE);
			dev_kfree_skb_any(p->rx_skb[i]);
		}
	}
	wr(p, PDMA_TX_BASE, 0);
	wr(p, PDMA_RX_BASE, 0);
}

static const struct of_device_id mt_match[] = {
	{ .compatible = "mediatek,mt7510-eth" },
	{}
};
MODULE_DEVICE_TABLE(of, mt_match);

static struct platform_driver mt_driver = {
	.probe = mt_probe,
	.remove = mt_remove,
	.driver = {
		.name = "mt7510_eth",
		.of_match_table = mt_match,
	},
};
module_platform_driver(mt_driver);

MODULE_DESCRIPTION("MT7510 PDMA ethernet (bring-up)");
MODULE_LICENSE("GPL");
