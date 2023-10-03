/**
 * Copyright (c) 2018 Metempsy Technology Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/cache/prefetch/pa_signature_path.hh"

#include <cassert>
#include <climits>

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/SignaturePathPrefetcher.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch
{

PASignaturePath::PASignaturePath(const SignaturePathPrefetcherParams &p)
    : SignaturePath(p)
{
}

void
PASignaturePath::calculatePrefetch(const PrefetchInfo &pfi,
                                 std::vector<AddrPriority> &addresses)
{
    Addr request_addr = pfi.getPaddr();
    Addr vpn = pfi.getAddr() / pageBytes;
    Addr ppn = request_addr / pageBytes;
    stride_t current_block = (request_addr % pageBytes) / blkSize;
    stride_t stride;
    bool is_secure = pfi.isSecure();
    double initial_confidence = 1.0;

    // Get the SignatureEntry of this page to:
    // - compute the current stride
    // - obtain the current signature of accesses
    bool miss;
    SignatureEntry &signature_entry = getSignatureEntry(ppn, is_secure,
            current_block, miss, stride, initial_confidence);

    if (miss) {
        // No history for this page, can't continue
        return;
    }

    if (stride == 0) {
        // Can't continue with a stride 0
        return;
    }

    // Update the confidence of the current signature
    updatePatternTable(signature_entry.signature, stride);

    // Update the current SignatureEntry signature
    signature_entry.signature =
        updateSignature(signature_entry.signature, stride);

    signature_t current_signature = signature_entry.signature;
    double current_confidence = initial_confidence;
    stride_t current_stride = signature_entry.lastBlock;

    // Look for prefetch candidates while the current path confidence is
    // high enough
    while (current_confidence > lookaheadConfidenceThreshold) {
        // With the updated signature, attempt to generate prefetches
        // - search the PatternTable and select all entries with enough
        //   confidence, these are prefetch candidates
        // - select the entry with the highest counter as the "lookahead"
        PatternEntry *current_pattern_entry =
            patternTable.findEntry(current_signature, false);
        PatternStrideEntry const *lookahead = nullptr;
        if (current_pattern_entry != nullptr) {
            unsigned long max_counter = 0;
            for (auto const &entry : current_pattern_entry->strideEntries) {
                //select the entry with the maximum counter value as lookahead
                if (max_counter < entry.counter) {
                    max_counter = entry.counter;
                    lookahead = &entry;
                }
                double prefetch_confidence =
                    calculatePrefetchConfidence(*current_pattern_entry, entry);

                if (prefetch_confidence >= prefetchConfidenceThreshold) {
                    assert(entry.stride != 0);
                    //prefetch candidate
                    addPrefetch(vpn, current_stride, entry.stride,
                                current_confidence, current_signature,
                                is_secure, addresses);
                }
            }
        }

        if (lookahead != nullptr) {
            current_confidence *= calculateLookaheadConfidence(
                    *current_pattern_entry, *lookahead);
            current_signature =
                updateSignature(current_signature, lookahead->stride);
            current_stride += lookahead->stride;
        } else {
            current_confidence = 0.0;
        }
    }

    auxiliaryPrefetcher(vpn, current_block, is_secure, addresses);
}

void
PASignaturePath::addPrefetch(Addr vpn, stride_t last_block,
    stride_t delta, double path_confidence, signature_t signature,
    bool is_secure, std::vector<AddrPriority> &addresses)
{
    stride_t block = last_block + delta;

    Addr pf_ppn;
    stride_t pf_block;
    if (block < 0) {
        stride_t num_cross_pages = 1 + (-block) / (pageBytes/blkSize);
        if (num_cross_pages > vpn) {
            // target address smaller than page 0, ignore this request;
            return;
        }
        pf_ppn = vpn - num_cross_pages;
        pf_block = block + (pageBytes/blkSize) * num_cross_pages;
        handlePageCrossingLookahead(signature, last_block, delta,
                                    path_confidence);
        return; // Cannot handle page crossing in PA prefetcher
    } else if (block >= (pageBytes/blkSize)) {
        stride_t num_cross_pages = block / (pageBytes/blkSize);
        if (MaxAddr/pageBytes < (vpn + num_cross_pages)) {
            // target address goes beyond MaxAddr, ignore this request;
            return;
        }
        pf_ppn = vpn + num_cross_pages;
        pf_block = block - (pageBytes/blkSize) * num_cross_pages;
        handlePageCrossingLookahead(signature, last_block, delta,
                                    path_confidence);
        return; // Cannot handle page crossing in PA prefetcher
    } else {
        pf_ppn = vpn;
        pf_block = block;
    }

    Addr new_addr = pf_ppn * pageBytes;
    new_addr += pf_block * (Addr)blkSize;

    DPRINTF(HWPrefetch, "Queuing prefetch to %#x.\n", new_addr);
    addresses.push_back(AddrPriority(new_addr, 0));
}

} // namespace prefetch
} // namespace gem5
