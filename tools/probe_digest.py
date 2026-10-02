import hashlib
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

enp = open(r'..\Vita_104_Firmware\Out\SLB2\second_loader.enp', 'rb').read()
rom = open(r'..\dumps\vita_prototype_bootrom.bin', 'rb').read()
romb = lambda a, n: rom[a - 0x5C000:a - 0x5C000 + n]

target = hashlib.sha256(enp[0x2C0:0x2C0 + 0x16600]).digest()
print('target =', target.hex())

# Every 32-byte block of the ENP header that the loader touches.
blocks = {
    'img+E0': enp[0xE0:0x100],
    'img+100': enp[0x100:0x120],
    'img+120': enp[0x120:0x140],
    'img+140': enp[0x140:0x160],
    'img+160': enp[0x160:0x180],
    'img+180': enp[0x180:0x1A0],
    'img+1A0': enp[0x1A0:0x1C0],
}
keys = {
    'img+E0': enp[0xE0:0x100],
    'img+E0a': enp[0xE0:0xF0],
    'img+E0b': enp[0xF0:0x100],
    'img+1A0': enp[0x1A0:0x1C0],
    'rom5E744': romb(0x5E744, 32),
    'rom5E744a': romb(0x5E744, 16),
    'rom5E764': romb(0x5E764, 32),
    'zero': bytes(32),
}


def aes(key, data, encrypt, mode='ecb', iv=bytes(16)):
    a = algorithms.AES(key)
    m = modes.ECB() if mode == 'ecb' else modes.CBC(iv)
    c = Cipher(a, m).encryptor() if encrypt else Cipher(a, m).decryptor()
    return c.update(data) + c.finalize()


hits = []
for bname, blk in blocks.items():
    if len(blk) != 32:
        continue
    for kname, key in keys.items():
        for klen in (32, 16):
            if len(key) < klen:
                continue
            k = key[:klen]
            for enc in (False, True):
                for mode in ('ecb', 'cbc'):
                    try:
                        out = aes(k, blk, enc, mode)
                    except Exception:
                        continue
                    if out == target:
                        hits.append('AES-%d-%s-%s key=%s' % (klen * 8, 'enc' if enc else 'dec', mode, kname))
                    if hashlib.sha256(out).digest() == target:
                        hits.append('sha256(AES-%d-%s-%s key=%s)' % (klen * 8, 'enc' if enc else 'dec', mode, kname))
    if hashlib.sha256(blk).digest() == target:
        hits.append('sha256(' + bname + ')')

# also: is the target simply one of the blocks, or the xor of two?
for bname, blk in blocks.items():
    if blk == target:
        hits.append('direct ' + bname)

print('hits:', hits)
