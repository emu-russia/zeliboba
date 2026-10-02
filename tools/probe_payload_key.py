import hashlib
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

enp = open(r'..\Vita_104_Firmware\Out\SLB2\second_loader.enp', 'rb').read()
enc = open(r'..\Vita_104_Firmware\Out\SLB2\second_loader.enc', 'rb').read()
rom = open(r'..\dumps\vita_prototype_bootrom.bin', 'rb').read()
romb = lambda a, n: rom[a - 0x5C000:a - 0x5C000 + n]

L = 0x16600
KEYS = {
    'enp+E0(32)': enp[0xE0:0x100],
    'enc+E0(32)': enc[0xE0:0x100],
    'enp+E0(16)': enp[0xE0:0xF0],
    'enc+E0(16)': enc[0xE0:0xF0],
    'rom5E744(32)': romb(0x5E744, 32),
    'rom5E744(16)': romb(0x5E744, 16),
}
IVS = {'zero': bytes(16), 'enp+F0': enp[0xF0:0x100], 'enc+F0': enc[0xF0:0x100],
       'enp+E0': enp[0xE0:0xF0], 'enc+E0': enc[0xE0:0xF0]}

for src_name, src in (('enp', enp), ('enc', enc)):
    payload = src[0x2C0:0x2C0 + L]
    for kname, key in KEYS.items():
        for klen in (32, 16):
            if len(key) < klen:
                continue
            k = key[:klen]
            variants = [('ecb', None)] + [('cbc', iv) for iv in IVS.values()]
            for mode, iv in variants:
                c = Cipher(algorithms.AES(k), modes.ECB() if iv is None else modes.CBC(iv)).decryptor()
                out = c.update(payload) + c.finalize()
                head = out[:16]
                # plausible MeP prologue starts: d0 6f (add sp,-N), 1a 70 (ldc $0,$lp), 80 6f, ...
                score = 1 if head[0] in (0xD0, 0x80, 0x1A, 0x00, 0x21) else 0
                if head[:2] in (b'\xd0\x6f', b'\x80\x6f', b'\x1a\x70'):
                    print('PLAUSIBLE PROLOGUE: %s key=%s mode=%s iv=%s -> %s' %
                          (src_name, kname, mode, 'zero' if iv is None else iv.hex()[:8], out[:32].hex(' ')))
                if score:
                    print('candidate %s key=%s-%d mode=%s iv=%s head=%s' %
                          (src_name, kname, klen * 8, mode, 'zero' if iv is None else iv.hex()[:8],
                           out[:16].hex(' ')))
