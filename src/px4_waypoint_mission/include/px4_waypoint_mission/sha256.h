#ifndef PX4_WAYPOINT_MISSION__SHA256_H_
#define PX4_WAYPOINT_MISSION__SHA256_H_

// Implementasi SHA-256 minimal, self-contained (tanpa dependency OpenSSL),
// dipakai untuk hash mission/canonical config (GAP_IMPLEMENTASI_DAN_ROADMAP.md
// §10). Bukan untuk keperluan kriptografi rahasia; hanya untuk deteksi
// perubahan/audit integritas file dan konfigurasi.

#include <array>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace px4_waypoint_mission
{

class Sha256
{
public:
  static std::string hash_hex(const std::string & data)
  {
    Sha256 hasher;
    hasher.update(reinterpret_cast<const uint8_t *>(data.data()), data.size());
    return hasher.finalize_hex();
  }

  void update(const uint8_t * data, std::size_t length)
  {
    for (std::size_t i = 0; i < length; ++i) {
      buffer_[buffer_length_++] = data[i];
      if (buffer_length_ == 64U) {
        transform(buffer_.data());
        bit_length_ += 512U;
        buffer_length_ = 0U;
      }
    }
  }

  std::string finalize_hex()
  {
    uint64_t total_bits = bit_length_ + static_cast<uint64_t>(buffer_length_) * 8U;
    buffer_[buffer_length_++] = 0x80U;
    if (buffer_length_ > 56U) {
      while (buffer_length_ < 64U) {buffer_[buffer_length_++] = 0x00U;}
      transform(buffer_.data());
      buffer_length_ = 0U;
    }
    while (buffer_length_ < 56U) {buffer_[buffer_length_++] = 0x00U;}
    for (int i = 7; i >= 0; --i) {
      buffer_[buffer_length_++] = static_cast<uint8_t>(total_bits >> (i * 8));
    }
    transform(buffer_.data());

    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (const auto & word : state_) {
      out << std::setw(8) << word;
    }
    return out.str();
  }

private:
  static uint32_t rotr(const uint32_t x, const uint32_t n) {return (x >> n) | (x << (32U - n));}

  void transform(const uint8_t * chunk)
  {
    static constexpr std::array<uint32_t, 64> k = {
      0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
      0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
      0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
      0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
      0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
      0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
      0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
      0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
      0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
      0xc67178f2};

    std::array<uint32_t, 64> w{};
    for (int i = 0; i < 16; ++i) {
      w[static_cast<std::size_t>(i)] =
        (static_cast<uint32_t>(chunk[i * 4]) << 24) |
        (static_cast<uint32_t>(chunk[i * 4 + 1]) << 16) |
        (static_cast<uint32_t>(chunk[i * 4 + 2]) << 8) |
        static_cast<uint32_t>(chunk[i * 4 + 3]);
    }
    for (int i = 16; i < 64; ++i) {
      const uint32_t s0 = rotr(w[static_cast<std::size_t>(i - 15)], 7) ^
        rotr(w[static_cast<std::size_t>(i - 15)], 18) ^ (w[static_cast<std::size_t>(i - 15)] >> 3);
      const uint32_t s1 = rotr(w[static_cast<std::size_t>(i - 2)], 17) ^
        rotr(w[static_cast<std::size_t>(i - 2)], 19) ^ (w[static_cast<std::size_t>(i - 2)] >> 10);
      w[static_cast<std::size_t>(i)] = w[static_cast<std::size_t>(i - 16)] + s0 +
        w[static_cast<std::size_t>(i - 7)] + s1;
    }

    uint32_t a = state_[0];
    uint32_t b = state_[1];
    uint32_t c = state_[2];
    uint32_t d = state_[3];
    uint32_t e = state_[4];
    uint32_t f = state_[5];
    uint32_t g = state_[6];
    uint32_t h = state_[7];

    for (int i = 0; i < 64; ++i) {
      const uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const uint32_t ch = (e & f) ^ ((~e) & g);
      const uint32_t temp1 = h + s1 + ch + k[static_cast<std::size_t>(i)] +
        w[static_cast<std::size_t>(i)];
      const uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t temp2 = s0 + maj;
      h = g; g = f; f = e; e = d + temp1;
      d = c; c = b; b = a; a = temp1 + temp2;
    }

    state_[0] += a; state_[1] += b; state_[2] += c; state_[3] += d;
    state_[4] += e; state_[5] += f; state_[6] += g; state_[7] += h;
  }

  std::array<uint32_t, 8> state_{
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  std::array<uint8_t, 64> buffer_{};
  std::size_t buffer_length_{0U};
  uint64_t bit_length_{0U};
};

}  // namespace px4_waypoint_mission

#endif  // PX4_WAYPOINT_MISSION__SHA256_H_
