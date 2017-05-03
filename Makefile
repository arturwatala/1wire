#
# Copyright (C) 2006-2015 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=1wire
PKG_VERSION:=1.0.1
PKG_RELEASE:=1
PKG_MAINTAINER:=Artur Watala 
PKG_LICENSE:=GPL-2
PKG_CONFIG_DEPENDS:=libconfig

include $(INCLUDE_DIR)/package.mk

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)-$(PKG_VERSION)

TARGET_LDFLAGS+= \
  -Wl,-rpath-link=$(STAGING_DIR)/usr/lib \
  -Wl,-rpath-link=$(STAGING_DIR)/usr/lib/libconfig/lib \
  -Wl,-rpath-link=$(STAGING_DIR)/usr/lib/sqlite/lib

define Package/1wire
  SECTION:=utils
  CATEGORY:=Utilities
  DEPENDS:=+libconfig
  TITLE:=1wire example program
  URL:=https://github.com/arturwatala/1wire
  MENU:=1
endef

define Package/1wire/description
 Load data from 1wire temprature sensor and send it over network.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Configure
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) $(TARGET_CONFIGURE_OPTS)
endef

define Package/1wire/install
	$(INSTALL_DIR) $(1)/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/1wire $(1)/bin/
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) $(PKG_BUILD_DIR)/1wire.conf $(1)/etc/config/1wire
endef

$(eval $(call BuildPackage,1wire))
