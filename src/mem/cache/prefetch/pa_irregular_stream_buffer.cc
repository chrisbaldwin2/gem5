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

#include "mem/cache/prefetch/pa_irregular_stream_buffer.hh"

#include "debug/HWPrefetch.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"
#include "params/IrregularStreamBufferPrefetcher.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch
{

PAIrregularStreamBuffer::PAIrregularStreamBuffer(
    const IrregularStreamBufferPrefetcherParams &p)
  : IrregularStreamBuffer(p)
{
}

void
PAIrregularStreamBuffer::calculatePrefetch(const PrefetchInfo &pfi,
    std::vector<AddrPriority> &addresses)
{
    // This prefetcher requires a PC
    if (!pfi.hasPC()) {
        return;
    }
    bool is_secure = pfi.isSecure();
    Addr pc = pfi.getPC();
    Addr vAddr = blockIndex(pfi.getAddr());
    Addr addr = blockIndex(pfi.getPaddr());

    // Training, if the entry exists, then we found a correlation between
    // the entry lastAddress (named as correlated_addr_A) and the address of
    // the current access (named as correlated_addr_B)
    TrainingUnitEntry *entry = trainingUnit.findEntry(pc, is_secure);
    bool correlated_addr_found = false;
    Addr correlated_addr_A = 0;
    Addr correlated_addr_B = 0;
    if (entry != nullptr && entry->lastAddressSecure == is_secure) {
        trainingUnit.accessEntry(entry);
        correlated_addr_found = true;
        correlated_addr_A = entry->lastAddress;
        correlated_addr_B = addr;
    } else {
        entry = trainingUnit.findVictim(pc);
        assert(entry != nullptr);

        trainingUnit.insertEntry(pc, is_secure, entry);
    }
    // Update the entry
    entry->lastAddress = addr;
    entry->lastAddressSecure = is_secure;

    if (correlated_addr_found) {
        // If a correlation was found, update the Physical-to-Structural
        // table accordingly
        AddressMapping &mapping_A = getPSMapping(correlated_addr_A, is_secure);
        AddressMapping &mapping_B = getPSMapping(correlated_addr_B, is_secure);
        if (mapping_A.counter > 0 && mapping_B.counter > 0) {
            // Entry for A and B
            if (mapping_B.address == (mapping_A.address + 1)) {
                mapping_B.counter++;
            } else {
                if (mapping_B.counter == 1) {
                    // Counter would hit 0, reassign address while keeping
                    // counter at 1
                    mapping_B.address = mapping_A.address + 1;
                    addStructuralToPhysicalEntry(mapping_B.address, is_secure,
                            correlated_addr_B);
                } else {
                    mapping_B.counter--;
                }
            }
        } else {
            if (mapping_A.counter == 0) {
                // if A is not valid, generate a new structural address
                mapping_A.counter++;
                mapping_A.address = structuralAddressCounter;
                structuralAddressCounter += chunkSize;
                addStructuralToPhysicalEntry(mapping_A.address,
                        is_secure, correlated_addr_A);
            }
            mapping_B.counter.reset();
            mapping_B.counter++;
            mapping_B.address = mapping_A.address + 1;
            // update SP-AMC
            addStructuralToPhysicalEntry(mapping_B.address, is_secure,
                    correlated_addr_B);
        }
    }

    // Use the PS mapping to predict future accesses using the current address
    // - Look for the structured address
    // - if it exists, use it to generate prefetches for the subsequent
    //   addresses in ascending order, as many as indicated by the degree
    //   (given the structured address S, prefetch S+1, S+2, .. up to S+degree)
    Addr amc_address = addr / prefetchCandidatesPerEntry;
    Addr map_index   = addr % prefetchCandidatesPerEntry;
    AddressMappingEntry *ps_am = psAddressMappingCache.findEntry(amc_address,
                                                                 is_secure);
    if (ps_am != nullptr) {
        AddressMapping &mapping = ps_am->mappings[map_index];
        if (mapping.counter > 0) {
            Addr sp_address = mapping.address / prefetchCandidatesPerEntry;
            Addr sp_index   = mapping.address % prefetchCandidatesPerEntry;
            AddressMappingEntry *sp_am =
                spAddressMappingCache.findEntry(sp_address, is_secure);
            if (sp_am == nullptr) {
                // The entry has been evicted, can not generate prefetches
                return;
            }
            for (unsigned d = 1;
                    d <= degree && (sp_index + d) < prefetchCandidatesPerEntry;
                    d += 1)
            {
                AddressMapping &spm = sp_am->mappings[sp_index + d];
                //generate prefetch
                if (spm.counter > 0) {
                    Addr pf_delta = addr - (spm.address << lBlkSize);
                    Addr pf_addr = vAddr + pf_delta;
                    // Mask if outside the page
                    if (pf_delta < pageBytes)
                        addresses.push_back(AddrPriority(pf_addr, 0));
                }
            }
        }
    }
}

} // namespace prefetch
} // namespace gem5
