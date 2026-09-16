// SketchSoC RTL prototype -- shared package.
//
// Paper: "SketchSoC: Query-Driven In-Switch Telemetry with Live
// Reconfiguration and Tiered State".  This package fixes the architectural
// constants (paper Sec. 3.4.3) and the packed record formats used across
// the design:
//
//   * state block size          256 bytes
//   * directory entry           64 bits {tier, addr, generation, owner}
//   * heat counter              16 bits per block (EWMA)
//   * replay queue              32-entry circular buffer of 128-bit
//                               logical update tokens, from a bounded
//                               shared pool (migration slots), not per block
//   * SMU hash families         MurmurHash3-derived and CRC32 (paper Sec. 3.4.1)
//
// One clock domain, active-low reset, simple valid/ready handshakes.

`timescale 1ns/1ps

`ifndef SKETCHSOC_PKG_SV
`define SKETCHSOC_PKG_SV

package sketchsoc_pkg;

  // ------------------------------------------------------------------
  // Architectural constants (paper Sec. 3.4.3)
  // ------------------------------------------------------------------
  localparam int unsigned BLOCK_BYTES  = 256;                       // state block size
  localparam int unsigned L0_WORD_B    = 4;                          // L0 word = 32 bit
  localparam int unsigned BLOCK_WORDS  = BLOCK_BYTES / L0_WORD_B;    // 64 words per block
  localparam int unsigned TOKEN_BITS   = 128;                        // 16-byte logical token

  localparam int unsigned QNUM         = 4;      // query slots (max 8: qid is 3 bit)
  localparam int unsigned QID_W        = 3;
  localparam int unsigned BLK_IDX_W    = 16;     // logical block index per query
  localparam int unsigned MAX_ROWS     = 8;      // CMS/HH rows (bounded by verifier)
  localparam int unsigned MAX_BLM_K    = 4;      // bloom hashes
  localparam int unsigned MAX_CAND     = 16;     // HH candidate table entries
  localparam int unsigned MIG_SLOTS    = 8;      // bounded shared migration pool
  localparam int unsigned FIFO_DEPTH   = 32;     // per-slot replay FIFO depth

  // ------------------------------------------------------------------
  // Tiers and owner states (paper Sec. 3.6.3)
  // ------------------------------------------------------------------
  typedef enum logic [1:0] {
    TIER_L0   = 2'd0,
    TIER_L1   = 2'd1,
    TIER_L2   = 2'd2
  } tier_e;

  typedef enum logic [1:0] {
    OWN_RESIDENT  = 2'd0,   // current tier applies updates directly
    OWN_MIGRATING = 2'd1,   // source authoritative, updates divert to FIFO
    OWN_REPLAYING = 2'd2,   // replay passes in progress
    OWN_COMMIT    = 2'd3    // brief commit window
  } owner_e;

  // ------------------------------------------------------------------
  // Sketch kinds (paper Table 2)
  // ------------------------------------------------------------------
  typedef enum logic [7:0] {
    KIND_NONE = 8'd0,
    KIND_CMS  = 8'd1,   // HASH, ADD, MIN     (conservative update)
    KIND_HH   = 8'd2,   // CMS + bounded candidates (CMP, SELECT)
    KIND_HLL  = 8'd3,   // HASH, CLZ, MAX
    KIND_ENT  = 8'd4,   // HASH, ADD
    KIND_BLM  = 8'd5    // HASH, OR
  } kind_e;

  typedef enum logic [1:0] {
    WID_8  = 2'd0,
    WID_16 = 2'd1,
    WID_32 = 2'd2
  } width_e;

  // ------------------------------------------------------------------
  // Flow key (5-tuple) and packet descriptor
  // ------------------------------------------------------------------
  typedef struct packed {
    logic [31:0] src_ip;
    logic [31:0] dst_ip;
    logic [15:0] src_port;
    logic [15:0] dst_port;
    logic [7:0]  proto;
    logic [7:0]  pad;
  } flow_key_t;                                  // 96 bit

  typedef struct packed {
    flow_key_t   key;                            // 96
    logic [31:0] pkt_id;                         // TB correlation
    logic [63:0] tstamp;                         // ingress timestamp (free-running)
    logic [95:0] rsv;
  } pkt_desc_t;                                  // 256 bit, one flit per packet

  // ------------------------------------------------------------------
  // 128-bit logical update token (paper Sec. 3.4.3 / 3.6.3)
  //
  // A token carries the *operation*, not a physical address: replay
  // re-hashes the key against the query configuration, so the same
  // execution engine serves both the packet path and the replay path.
  // ------------------------------------------------------------------
  typedef struct packed {
    flow_key_t   key;                            // [127:32]
    logic [7:0]  kind;                           // [31:24]
    logic [2:0]  qid;                            // [23:21]
    logic [19:0] param;                          // [20:0]  (reserved)
  } token_t;

  // ------------------------------------------------------------------
  // Directory entry: 64 bit (paper Sec. 3.4.3)
  // ------------------------------------------------------------------
  typedef struct packed {
    logic        valid;                          // [63]
    logic [1:0]  tier;                           // [62:61]
    logic [1:0]  owner;                          // [60:59]
    logic [7:0]  gen;                            // [58:51] block generation
    logic [3:0]  mslot;                          // [50:47] migration slot while frozen
    logic [31:0] addr;                           // [46:15] L0: word base | L1: byte addr
    logic [14:0] rsv;                            // [14:0]
  } dirent_t;

  // ------------------------------------------------------------------
  // Query descriptor -- the "dynamic configuration state" of Fig. 5.
  // Online reconfiguration rewrites these registers (install / remove /
  // re-parameterise) without resynthesis.
  // ------------------------------------------------------------------
  typedef struct packed {
    logic        enable;
    logic [7:0]  kind;         // kind_e
    logic [3:0]  rows;         // CMS/HH rows (1..MAX_ROWS)
    logic [5:0]  cols_log2;    // cells per row = 2^cols_log2
    logic [1:0]  width;        // width_e (8/16/32 bit counters)
    logic [31:0] seed0;        // hash seeds
    logic [31:0] seed1;
    logic [31:0] seed2;
    logic [31:0] seed3;
    logic [4:0]  cand_log2;    // HH: candidate count = 2^cand_log2 (<= MAX_CAND)
    logic [2:0]  blm_k;        // bloom: number of hashes (1..MAX_BLM_K)
    logic [7:0]  gen;          // configuration epoch/generation
  } qdesc_t;

  // ------------------------------------------------------------------
  // Migration slot command (tier manager -> migration engine)
  // ------------------------------------------------------------------
  typedef enum logic [1:0] {
    MIG_PROMOTE = 2'd0,      // L1 -> L0
    MIG_DEMOTE  = 2'd1       // L0 -> L1
  } mig_op_e;

  typedef struct packed {
    logic [1:0]  op;          // mig_op_e
    logic [2:0]  qid;
    logic [15:0] block;       // logical block index
    logic [31:0] dest_addr;   // destination base: L0 word base | L1 byte base
  } mig_cmd_t;

  // ------------------------------------------------------------------
  // Hash families (paper Sec. 3.4.1: CRC32 / MurmurHash3 / xxHash-derived)
  // Combinational; each is a distinct index function family so that
  // sketch rows are independent.
  // ------------------------------------------------------------------
  function automatic logic [31:0] fmix32(input logic [31:0] h);
    h ^= h >> 16;
    h *= 32'h85ebca6b;
    h ^= h >> 13;
    h *= 32'hc2b2ae35;
    h ^= h >> 16;
    return h;
  endfunction

  // MurmurHash3-derived: mix the 96-bit key one 32-bit word at a time.
  function automatic logic [31:0] hash_murmur(input logic [31:0] seed,
                                              input flow_key_t key);
    logic [31:0] h;
    h  = seed;
    h  = fmix32(h ^ key.src_ip);
    h  = fmix32(h ^ key.dst_ip);
    h  = fmix32(h ^ {key.src_port, key.dst_port, key.proto});
    return fmix32(h);
  endfunction

  // CRC32 (reflected, poly 0xEDB88320) over {seed, key}: bit-serial
  // formulation unrolls into a flat XOR network in synthesis.
  function automatic logic [31:0] hash_crc32(input logic [31:0] seed,
                                             input flow_key_t key);
    logic [127:0] d;
    logic [31:0] c;
    int unsigned i;
    d = {key, seed};
    c = 32'hFFFFFFFF;
    for (i = 0; i < 128; i++) begin
      if ((c[0] ^ d[i]) != 1'b0)
        c = {1'b0, c[31:1]} ^ 32'hEDB88320;
      else
        c = {1'b0, c[31:1]};
    end
    return ~c;
  endfunction

  // xxHash-derived avalanche over the key with the seed folded in.
  function automatic logic [31:0] hash_xx(input logic [31:0] seed,
                                          input flow_key_t key);
    logic [31:0] h;
    h  = seed + 32'h9E3779B1;
    h  = fmix32(h ^ key.src_ip);
    h  = fmix32(h ^ key.dst_ip);
    h  = fmix32(h ^ {key.src_port, key.dst_port, key.proto});
    return h;
  endfunction

  // Row hash dispatch: row r of a query uses a distinct family/seed pair,
  // so no two rows of the same query collide systematically.
  function automatic logic [31:0] row_hash(input int unsigned r,
                                           input logic [1:0]  fam,
                                           input logic [31:0] s0,
                                           input logic [31:0] s1,
                                           input logic [31:0] s2,
                                           input logic [31:0] s3,
                                           input flow_key_t   key);
    logic [31:0] s;
    s = (r == 0) ? s0 : (r == 1) ? s1 : (r == 2) ? s2 : s3 ^ (32'h9E3779B1 * r);
    case (fam)
      2'd0: return hash_murmur(s, key);
      2'd1: return hash_crc32 (s, key);
      default: return hash_xx  (s, key);
    endcase
  endfunction

  // Count leading zeros (HLL rank).  Returns 1..33 as the HLL bucket rank
  // for a 32-bit hash (rank 33 saturates: zero hash).
  function automatic logic [7:0] clz32(input logic [31:0] h);
    logic [7:0] n;
    n = 8'd1;
    for (int i = 31; i >= 0; i--) begin   // signed: i>=0 must terminate (synth unroll)
      if (h[i] == 1'b0) n = n + 8'd1;
      else break;
    end
    if (n > 8'd33) n = 8'd33;
    return n;
  endfunction

  // Cell geometry helpers ------------------------------------------------
  // cells per 256-byte block by width
  function automatic int unsigned cells_per_block(input logic [1:0] w);
    case (w)
      WID_8:  return BLOCK_BYTES / 1;
      WID_16: return BLOCK_BYTES / 2;
      default: return BLOCK_BYTES / 4;
    endcase
  endfunction

  // byte enable for a cell of width w at byte offset bofs within a 32-bit word
  function automatic logic [3:0] cell_strb(input logic [1:0] w,
                                           input logic [1:0] bofs);
    case (w)
      WID_8:  return 4'b0001 << bofs;
      WID_16: return (bofs[0] == 1'b0) ? 4'b0011 : 4'b1100;
      default: return 4'b1111;
    endcase
  endfunction

endpackage : sketchsoc_pkg

`endif // SKETCHSOC_PKG_SV
