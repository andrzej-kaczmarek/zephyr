/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Enables CTE transmission according to provided configuration */
void lll_df_cte_tx_enable(struct lll_adv_sync *lll_sync,
			  const struct pdu_adv *pdu,
			  uint32_t *cte_len_us);
/* Disables CTE transmission */
void lll_df_conf_cte_tx_disable(void);
