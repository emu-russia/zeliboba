# Native ThreadMgr software thread registers

Native proof is recorded in `goal-native-tpidr-context-evidence.md` and
`goal-arm-native-tpidr-context.log/.json`. ThreadMgr writes TPIDRPRW at
VA `0x004A1DE8`, Thumb32 bytes `0D EE 90 9F` (MCR p15,0,R9,c13,c0,4):

| Core | Guest thread pointer |
| --- | --- |
| ARM0 | 0x0006BA28 |
| ARM1 | 0x0006BC28 |
| ARM2 | 0x0006BE28 |
| ARM3 | 0x0006B828 |

The old CPU discarded those writes and returned zero for TPIDRPRW reads.
Native mutex/event-flag creation consequently returned `0x80027101`
(ILLEGAL_CONTEXT). The reached mutex check also reads and writes TPIDRURO,
so implementing only the privileged thread pointer would leave part of the
same architectural context protocol absent.

Primary ARM ARM DDI0406C.d B4.1.150..152, printed B4-1714..1715 (local
`build/research/zeliboba-review-thumb-it/arm-ddi0406cd.txt` lines 91847..91960), defines
three software-only 32-bit registers, banked with Security Extensions:
TPIDRURW (Op2=2, all privilege levels read/write), TPIDRURO (Op2=3, PL0
read-only), and TPIDRPRW (Op2=4, PL1+ read/write). Their exact encoding is
Op1=0, CRn=c13, CRm=c0. Hardware never updates them; reset is UNKNOWN.
The emulator chooses zero in both banks for deterministic reset.

B3.15.3/Table B3-33, B3-1448, and the Monitor-access rules B3-1455..1456
(local lines 76655 and 77170 onward) distinguish execution security from
register access. Monitor executes Secure, while its MRC/MCR selects the
banked register copy with SCR.NS. The implementation follows that rule.

Each core now retains all three registers in separate Secure/NS banks.
Both A32 and Thumb32 enforce the thread-register privilege matrix before
performing a transfer, so a forbidden read also preserves its destination.
The debugger exposes the registers using the same CP15 access-bank selector.
Other CP15 registers and instruction families are outside this change.

Three meaningful CPU tests cover the unchanged native four-core writer and
mutex context-gate bytes, A32/Thumb32 user permissions in both security
states, actual SMC Monitor-bank reads and exception return, register-bank
independence, non-alias encodings, and reset. The native gate failed **24
assertions** against the original 548-test CPU and now passes, taking the
nonzero-context branch with each core's genuine pointer.

Logs: `goal-native-tpidr-cpu-before.log`, `goal-native-tpidr-cpu-tests.log`,
and `goal-native-tpidr-inspection-tests.log`. Combined isolated validation
passes **171 tests / 0 failures** (167 ARM and 4 debugger). Six new cases
across debugger/TPIDR imply integrated **554**, before other agents' tests.
CPU sources are frozen. Full native I2C/boot progress awaits integration.
