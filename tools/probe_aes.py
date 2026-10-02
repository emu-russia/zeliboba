import hashlib
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

enp = open(r'..\Vita_104_Firmware\Out\SLB2\second_loader.enp', 'rb').read()
enc = open(r'..\Vita_104_Firmware\Out\SLB2\second_loader.enc', 'rb').read()
rom = open(r'..\dumps\vita_prototype_bootrom.bin', 'rb').read()
romb = lambda a, n: rom[a - 0x5C000:a - 0x5C000 + n]

target = hashlib.sha256(enp[0x2C0:0x2C0 + 0x16600]).digest()
print('target (payload sha256) =', target.hex())

src_full = enp[0x1A0:0x1C0]
keys = {
    'image+E0(32)': enp[0xE0:0x100],
    'image+E0(16)': enp[0xE0:0xF0],
    'rom5E744(32)': romb(0x5E744, 32),
    'rom5E744(16)': romb(0x5E744, 16),
}
ivs = {
    'image+E0(16)': enp[0xE0:0xF0],
    'image+F0(16)': enp[0xF0:0x100],
    'zero': bytes(16),
}
hits = []
for kname, key in keys.items():
    for mode in ('ecb', 'cbc'):
        for iname, iv in (ivs.items() if mode == 'cbc' else [('none', None)]):
            try:
                if mode == 'ecb':
                    c = Cipher(algorithms.AES(key), modes.ECB()).decryptor()
                else:
                    c = Cipher(algorithms.AES(key), modes.CBC(iv)).decryptor()
                out = c.update(src_full) + c.finalize()
            except Exception as exc:
                continue
            tag = '%s %s key=%s iv=%s' % (mode, 'dec', kname, iname)
            if out == target:
                hits.append(tag)
            if hashlib.sha256(out).digest() == target:
                hits.append(tag + ' [sha256 of output]')
            print('%-52s %s' % (tag, out.hex()))
print('HITS:', hits)
