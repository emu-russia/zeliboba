import hashlib
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

enp = open(r'..\Vita_104_Firmware\Out\SLB2\second_loader.enp', 'rb').read()
enc = open(r'..\Vita_104_Firmware\Out\SLB2\second_loader.enc', 'rb').read()
bin_ = open(r'..\Vita_104_Firmware\Out\SLB2_dec\second_loader.bin', 'rb').read()
rom = open(r'..\dumps\vita_prototype_bootrom.bin', 'rb').read()
romb = lambda a, n: rom[a - 0x5C000:a - 0x5C000 + n]

KEYS = {
    'aa*16': bytes([0xAA] * 16),
    'enp+E0': enp[0xE0:0xF0],
    'enp+F0': enp[0xF0:0x100],
    'enc+E0': enc[0xE0:0xF0],
    'enc+F0': enc[0xF0:0x100],
    'rom5E744': romb(0x5E744, 16),
    'rom5E754': romb(0x5E754, 16),
    'rom5E960': romb(0x5E960, 16),
}
IVS = {
    'af5f': bytes.fromhex('AF5F2CB04AC1751ABF51CEF1C8096210'),
    'zero': bytes(16),
    'enp+E0': enp[0xE0:0xF0],
    'enp+F0': enp[0xF0:0x100],
    'enc+E0': enc[0xE0:0xF0],
    'rom5E744h': romb(0x5E744, 16),
}

hits = []
for src_name, src in (('enp', enp), ('enc', enc)):
    payload = src[0x2C0:0x2C0 + 0x16600]
    for kname, key in KEYS.items():
        for klen in (16, 32):
            k = (key + bytes(32))[:klen] if len(key) < klen else key[:klen]
            cands = [('ecb', bytes(16))]
            cands += [('cbc', iv) for iv in IVS.values()]
            for mode, iv in cands:
                try:
                    c = Cipher(algorithms.AES(k), modes.ECB() if mode == 'ecb' else modes.CBC(iv)).decryptor()
                    out = c.update(payload) + c.finalize()
                except Exception:
                    continue
                if out == bin_:
                    hits.append('%s payload AES-%d-%s key=%s iv=%s' % (src_name, klen * 8, mode, kname, iv.hex()[:8]))
print('hits:', hits)
print('bin sha256:', hashlib.sha256(bin_).hexdigest())
