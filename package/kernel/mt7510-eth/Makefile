include $(TOPDIR)/rules.mk

PKG_NAME:=mt7510-eth
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

define KernelPackage/mt7510-eth
  SUBMENU:=Network Devices
  TITLE:=MT7510-class PDMA Ethernet Driver (chip id 0x0005xxxx)
  DEPENDS:=@TARGET_econet
  FILES:=$(PKG_BUILD_DIR)/mt7510_eth.ko
  AUTOLOAD:=$(call AutoLoad,90,mt7510_eth)
endef

define KernelPackage/mt7510-eth/description
  PDMA ethernet driver for the older MT7510-class frame engine that has
  no QDMA block (seen on ZTE ZXHN H267A hw V1.0.2). DSA conduit for the
  on-die MT7530.
endef

define Build/Compile
	$(KERNEL_MAKE) M=$(PKG_BUILD_DIR) modules
endef

$(eval $(call KernelPackage,mt7510-eth))
