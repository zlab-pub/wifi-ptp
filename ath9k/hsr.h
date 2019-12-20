/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Kirill Berezin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef HSR_H
#define HSR_H

#ifdef CPTCFG_ATH9K_UBNTHSR

void ath9k_hsr_init(struct ath_hw *ah);
int ath9k_hsr_disable(struct ath_hw *ah);
int ath9k_hsr_enable(struct ath_hw *ah, int bw, int fq);
int ath9k_hsr_status(struct ath_hw *ah);

#else
static inline void ath9k_hsr_init(struct ath_hw *ah) {}

static inline int ath9k_hsr_enable(struct ath_hw *ah, int bw, int fq)
{
	return 0;
}

static inline int ath9k_hsr_disable(struct ath_hw *ah) { return 0; }
static inline int ath9k_hsr_status(struct ath_hw *ah) { return 0; }

#endif

#endif /* HSR_H */
