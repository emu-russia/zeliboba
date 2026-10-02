import hashlib
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

enp = open(r'..\Vita_104_Firmware\Out\SLB2\second_loader.enp', 'rb').read()
enc = open(r'..\Vita_104_Firmware\Out\SLB2\second_loader.enc', 'rb').read()
rom = open(r'..\dumps\vita_prototype_bootrom.bin', 'rb').read()
romb = lambda a, n: rom[a - 0x5C000:a - 0x5C000 + n]

L = 0x16600
p_enp = enp[0x2C0:0x2C0 + L]
p_enc = enc[0x2C0:0x2C0 + L]
print('sha256(enp payload)', hashlib.sha256(p_enp).hexdigest())
print('sha256(enc payload)', hashlib.sha256(p_enc).hexdigest())
print('enp[0x1A0]', enp[0x1A0:0x1C0].hex())
print('enc[0x1A0]', enc[0x1A0:0x1C0].hex())

keys = {
    'rom5E744(32)': romb(0x5E744, 32),
    'rom5E744(16)': romb(0x5E744, 16),
    'enp+E0(32)': enp[0xE0:0x100],
    'enc+E0(32)': enc[0xE0:0x100],
    'enc+E0(16)': enc[0xE0:0xF0],
    'enp+E0(16)': enp[0xE0:0xF0],
    'rom5E960(16)': romb(0x5E960, 16),
    'rom5E960(32)': romb(0x5E960, 32),
}
hits = []
for kname, key in keys.items():
    for klen in (32, 16):
        if len(key) < klen:
            continue
        k = key[:klen]
        for mode in ('ecb', 'cbc'):
            for ivname, iv in ([('zero', bytes(16))] if mode == 'ecb' else [('zero', bytes(16))]):
                # is enc payload the encryption of the enp payload?
                c = Cipher(algorithms.AES(k), modes.ECB() if mode == 'ecb' else modes.CBC(iv)).encryptor()
                out = c.update(p_enp) + c.finalize()
                if out == p_enc:
                    hits.append('enc = AES-%s-ENC(enp) key=%s' % (mode, kname))
                # or the decryption?
                d = Cipher(algorithms.AES(k), modes.ECB() if mode == 'ecb' else modes.CBC(iv)).decryptor()
                out2 = d.update(p_enc) + d.finalize()
                if out2 == p_enp:
                    hits.append('enp = AES-%s-DEC(enc) key=%s' % (mode, kname))
                if hashlib.sha256(out2).hexdigest() in (enp[0x1A0:0x1C0].hex(), enc[0x1A0:0x1C0].hex()):
                    hits.append('sha256(AES-%s-DEC(enc)) matches a table entry key=%s' % (mode, kname))
print('hits:', hits)
