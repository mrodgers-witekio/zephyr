/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUPP_MGMT_H
#define ZEPHYR_SUPP_MGMT_H

#include <zephyr/net/wifi_mgmt.h>

#ifndef MAX_SSID_LEN
#define MAX_SSID_LEN 32
#endif
#ifndef MAC_ADDR_LEN
#define MAC_ADDR_LEN 6
#endif

#define MAC_STR_LEN 18 /* for ':' or '-' separated MAC address string */
#define CHAN_NUM_LEN 6 /* for space-separated channel numbers string */

/**
 * @brief Get version
 *
 * @param dev: Wi-Fi interface name to use
 * @param params: version to fill
 *
 * @return: 0 for OK; <0 for ERROR
 */
int supplicant_get_version(const struct device *dev, struct wifi_version *params);

/**
 * @brief Request a connection
 *
 * @param dev: Wi-Fi interface name to use
 * @param params: Connection details
 *
 * @return: 0 for OK; -1 for ERROR
 */
int supplicant_connect(const struct device *dev, struct wifi_connect_req_params *params);

/**
 * @brief Forces station to disconnect and stops any subsequent scan
 *  or connection attempts
 *
 * @param dev: Wi-Fi interface name to use
 *
 * @return: 0 for OK; -1 for ERROR
 */
int supplicant_disconnect(const struct device *dev);

/**
 * @brief
 *
 * @param dev: Wi-Fi interface name to use
 * @param status: Status structure to fill
 *
 * @return: 0 for OK; -1 for ERROR
 */
int supplicant_status(const struct device *dev, struct wifi_iface_status *status);

/**
 * @brief Request a scan
 *
 * @param dev Wi-Fi interface name to use
 * @param params Scan parameters
 * @param cb Callback to be called for each scan result
 *
 * @return 0 for OK; -1 for ERROR
 */
int supplicant_scan(const struct device *dev, struct wifi_scan_params *params,
		    scan_result_cb_t cb);

#if defined(CONFIG_NET_STATISTICS_WIFI) || defined(__DOXYGEN__)
/**
 * @brief Get Wi-Fi statistics
 *
 * @param dev Wi-Fi interface name to use
 * @param stats Pointer to stats structure to fill
 *
 * @return 0 for OK; -1 for ERROR
 */
int supplicant_get_stats(const struct device *dev, struct net_stats_wifi *stats);
#endif /* CONFIG_NET_STATISTICS_WIFI || __DOXYGEN__ */

/** Flush PMKSA cache entries
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @return 0 if ok, < 0 if error
 */
int supplicant_pmksa_flush(const struct device *dev);

/**
 * @brief Set Wi-Fi power save configuration
 *
 * @param dev Wi-Fi interface name to use
 * @param params Power save parameters to set
 *
 * @return 0 for OK; -1 for ERROR
 */
int supplicant_set_power_save(const struct device *dev, struct wifi_ps_params *params);

/**
 * @brief Set Wi-Fi TWT parameters
 *
 * @param dev Wi-Fi interface name to use
 * @param params TWT parameters to set
 * @return 0 for OK; -1 for ERROR
 */
int supplicant_set_twt(const struct device *dev, struct wifi_twt_params *params);

/**
 * @brief Get Wi-Fi power save configuration
 *
 * @param dev Wi-Fi interface name to use
 * @param config Address of power save configuration to fill
 * @return 0 for OK; -1 for ERROR
 */
int supplicant_get_power_save_config(const struct device *dev, struct wifi_ps_config *config);

/**
 * @brief Set Wi-Fi Regulatory domain
 *
 * @param dev Wi-Fi interface name to use
 * @param reg_domain Regulatory domain to set
 * @return 0 for OK; -1 for ERROR
 */
int supplicant_reg_domain(const struct device *dev, struct wifi_reg_domain *reg_domain);

/**
 * @brief Set Wi-Fi mode of operation
 *
 * @param dev Wi-Fi interface name to use
 * @param mode Mode setting to set
 * @return 0 for OK; -1 for ERROR
 */
int supplicant_mode(const struct device *dev, struct wifi_mode_info *mode);

#ifdef CONFIG_WIFI_NM_WPA_SUPPLICANT_CRYPTO_ENTERPRISE
/** Set Wi-Fi enterprise mode CA/client Cert and key
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param file Pointer to the CA/client Cert and key.
 *
 * @return 0 if ok, < 0 if error
 */
int supplicant_add_enterprise_creds(const struct device *dev,
		struct wifi_enterprise_creds_params *creds);
#endif

/**
 * @brief Set Wi-Fi packet filter for sniffing operation
 *
 * @param dev Wi-Fi interface name to use
 * @param filter Filter settings to set
 * @return 0 for OK; -1 for ERROR
 */
int supplicant_filter(const struct device *dev, struct wifi_filter_info *filter);

/**
 * @brief Set Wi-Fi channel for monitor or TX injection mode
 *
 * @param dev Wi-Fi interface name to use
 * @param channel Channel settings to set
 * @return 0 for OK; -1 for ERROR
 */
int supplicant_channel(const struct device *dev, struct wifi_channel_info *channel);

#ifdef CONFIG_WIFI_NM_WPA_SUPPLICANT_WNM
/** Send bss transition query
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param reason query reason
 *
 * @return 0 if ok, < 0 if error
 */
int supplicant_btm_query(const struct device *dev, uint8_t reason);
#endif

/** Get Wi-Fi connection parameters recently used
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param params the Wi-Fi connection parameters recently used
 *
 * @return 0 if ok, < 0 if error
 */
int supplicant_get_wifi_conn_params(const struct device *dev,
			 struct wifi_connect_req_params *params);

#ifdef CONFIG_AP
#ifdef CONFIG_WIFI_NM_HOSTAPD_AP
/**
 * @brief Get Wi-Fi AP Status
 *
 * @param dev Wi-Fi device
 * @param params AP status
 * @return 0 for OK; -1 for ERROR
 */
int hapd_state(const struct device *dev, int *state);
#else
static inline int hapd_state(const struct device *dev, int *state)
{
	return -EINVAL;
}
#endif

/**
 * @brief Set Wi-Fi AP configuration
 *
 * @param dev Wi-Fi interface name to use
 * @param params AP configuration parameters to set
 * @return 0 for OK; -1 for ERROR
 */
int supplicant_ap_enable(const struct device *dev,
			 struct wifi_connect_req_params *params);

/**
 * @brief Disable Wi-Fi AP
 * @param dev Wi-Fi interface name to use
 * @return 0 for OK; -1 for ERROR
 */
int supplicant_ap_disable(const struct device *dev);

/**
 * @brief Set Wi-Fi AP STA disconnect
 * @param dev Wi-Fi interface name to use
 * @param mac_addr MAC address of the station to disconnect
 * @return 0 for OK; -1 for ERROR
 */
int supplicant_ap_sta_disconnect(const struct device *dev,
				 const uint8_t *mac_addr);

#endif /* CONFIG_AP */

/**
 * @brief Dispatch DPP operations
 *
 * @param dev Wi-Fi interface name to use
 * @param dpp_params DPP action enum and params in string
 * @return 0 for OK; -1 for ERROR
 */
int supplicant_dpp_dispatch(const struct device *dev,
			    struct wifi_dpp_params *params);
#endif /* ZEPHYR_SUPP_MGMT_H */
