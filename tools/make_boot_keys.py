"""Generate the development RSA key that lets the first loader's verification pass.

The first loader (img_proc_5C798) verifies the staged image with a 2048-bit RSA
public key that on real hardware lives in the CMeP key table at 0xE0066000. That
table is not in any dump we have, so the emulator seeds the table with a *development*
key and re-signs the staged second loader with the matching private key. Every real
code path still runs - the loader really performs the exponentiation and the memcmp
against its own ROM constants.

What the loader expects (all constants read straight out of the proto boot ROM):

    buffer[  0..  1] = 00 01
    buffer[  2..204] = FF * 203
    buffer[205]      = 00
    buffer[206..223] = ROM 0x5E774 (18)   DigestInfo (hmacWithSHA256)
    buffer[224..255] = SHA-256(image[0:0x1C0])
    => s^e mod n == buffer,  e = 65537 (the LE word stream at ROM 0x5E764)

The tail is not a ROM constant.  0x5C7F6 hashes the image header - everything
before the 256-byte signature at image+0x1C0 - into 0x5EDC0, and 0x5C898 memcmps
that buffer against buffer[224:256].  So the signed block covers the header of the
staged second loader, which is why this script reads it.  The header includes the
digest table entry at +0x1A0 that machine/bootkeys.cpp step 1 fills in with
SHA-256 of the payload, so this script applies the same patch before hashing.

The loader copies the 64 words at image+0x1C0 into the engine's base window, so the
signature must be stored there in the engine's window order (little endian, word 0
least significant), and the modulus table at 0xE0066000 in the same order.
"""
import hashlib
import os
import random
import sys

ROOT = r'C:\Work\PSVita'
OUT = os.path.join(ROOT, 'zeliboba', 'src', 'machine', 'bootkeys_data.h')
IMAGE = os.path.join(ROOT, 'Vita_104_Firmware', 'Out', 'SLB2', 'second_loader.enc')
SIGNATURE_OFFSET = 0x1C0

rom = open(os.path.join(ROOT, 'dumps', 'vita_prototype_bootrom.bin'), 'rb').read()
BASE = 0x5C000


def rom_bytes(address, length):
    return rom[address - BASE:address - BASE + length]


D18 = rom_bytes(0x5E774, 18)
KEY20 = rom_bytes(0x5E744, 32)
EXP_WORD = int.from_bytes(rom_bytes(0x5E764, 4), 'little')

image = bytearray(open(IMAGE, 'rb').read())
PAYLOAD_OFFSET = 0x2C0
DIGEST_TABLE_OFFSET = 0x1A0
# machine/bootkeys.cpp step 1: the payload digest goes into the header's digest
# table before the header is hashed, so the signed block covers the patched header.
payload_length = int.from_bytes(image[0x10:0x14], 'little')
assert payload_length and PAYLOAD_OFFSET + payload_length <= len(image), payload_length
image[DIGEST_TABLE_OFFSET:DIGEST_TABLE_OFFSET + 32] = hashlib.sha256(
    bytes(image[PAYLOAD_OFFSET:PAYLOAD_OFFSET + payload_length])).digest()
HEADER_DIGEST = hashlib.sha256(bytes(image[:SIGNATURE_OFFSET])).digest()

MESSAGE = bytes([0x00, 0x01]) + b'\xFF' * 203 + bytes([0x00]) + D18 + HEADER_DIGEST
assert len(MESSAGE) == 256
m = int.from_bytes(MESSAGE, 'big')

print('DigestInfo (18):', D18.hex(' '))
print('keyring20 (32) :', KEY20.hex(' '))
print('payload bytes  : 0x%X' % payload_length)
print('header sha256  :', HEADER_DIGEST.hex())
print('exponent word  : 0x%08X' % EXP_WORD)
assert EXP_WORD == 65537, EXP_WORD


# --- deterministic RSA-2048 key generation -----------------------------------
def is_probable_prime(n, rng, rounds=40):
    if n < 2:
        return False
    for p in (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37):
        if n % p == 0:
            return n == p
    d = n - 1
    r = 0
    while d % 2 == 0:
        d //= 2
        r += 1
    for _ in range(rounds):
        a = rng.randrange(2, n - 1)
        x = pow(a, d, n)
        if x == 1 or x == n - 1:
            continue
        for _ in range(r - 1):
            x = x * x % n
            if x == n - 1:
                break
        else:
            return False
    return True


def gen_prime(bits, rng):
    while True:
        candidate = rng.getrandbits(bits) | (1 << (bits - 1)) | 1
        if is_probable_prime(candidate, rng):
            return candidate


rng = random.Random(0x5A17B0BA)          # fixed seed: the key is reproducible
E = 65537
while True:
    p = gen_prime(1024, rng)
    q = gen_prime(1024, rng)
    if p == q:
        continue
    n = p * q
    if n.bit_length() != 2048:
        continue
    phi = (p - 1) * (q - 1)
    if phi % E == 0:
        continue
    d = pow(E, -1, phi)
    break

print('modulus bitlen :', n.bit_length())
assert m < n

signature = pow(m, d, n)
check = pow(signature, E, n)
assert check == m, 'key self-test failed'
print('self-test      : s^e mod n == expected block  OK')
print('sha256(m)      :', hashlib.sha256(MESSAGE).hexdigest())

n_le = n.to_bytes(256, 'little')
s_le = signature.to_bytes(256, 'little')
n_be = n.to_bytes(256, 'big')
s_be = signature.to_bytes(256, 'big')


def carray(name, blob, per_line=12):
    lines = ['static const unsigned char %s[%d] = {' % (name, len(blob))]
    for i in range(0, len(blob), per_line):
        chunk = ', '.join('0x%02X' % b for b in blob[i:i + per_line])
        lines.append('    ' + chunk + ',')
    lines.append('};')
    return '\n'.join(lines)


text = '''// Generated by tools/make_boot_keys.py - do not edit by hand.
//
// A development RSA-2048 key pair used only because the console's real public key
// table (CMeP 0xE0066000) is not present in any dump we have. `modulus_window` is
// written into that table and `signature_window` replaces the 256 bytes at
// image+0x1C0 of the staged second loader, so the loader's own RSA verification
// succeeds through the normal hardware model.
//
// Signed message block (the loader's own ROM constants):
//   %s
//
#pragma once

namespace zlb {
namespace bootkeys {

/// e = 65537, the exponent the loader streams from ROM 0x5E764.
constexpr unsigned kExponent = 65537u;

/// The digest the loader compares against, used to refuse provisioning when the
/// fitted first loader is a different build than the one this key was made for.
%s

%s

%s

}  // namespace bootkeys
}  // namespace zlb
''' % (MESSAGE.hex(), carray('kExpectedMessageDigest', hashlib.sha256(MESSAGE).digest()),
       carray('kModulusWindow', n_le), carray('kSignatureWindow', s_le))

open(OUT, 'w', encoding='utf-8').write(text)
print('wrote', OUT, len(text), 'bytes')

# A copy for the record, and the big-endian variants in case the engine's window
# order turns out to be the other way round.
open(os.path.join(ROOT, 'zeliboba', 'build', 'boot_keys_le.bin'), 'wb').write(n_le + s_le)
open(os.path.join(ROOT, 'zeliboba', 'build', 'boot_keys_be.bin'), 'wb').write(n_be + s_be)
print('also wrote build/boot_keys_le.bin and build/boot_keys_be.bin')
