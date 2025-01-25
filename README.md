# A Self-Synchronizing, Low-Power, Low-Complexity Transceiver for Body-Coupled Communication (BCC)
Just imagine your body is being used as a communication medium. No wireless medium and no wire, instead, your body transfers data between the smart devices connected to your body. Also imagine using this technique to connect the worn devices while you are underwater.

**Note1:** The Arduino codes provided here are non-professional and are an example implementation. You may need to change them as per your needs. ECS1 is used in these codes. However, other ECS members can also be used.

**Note2:** Detailed text and pulications are at the bottom of the page. Following is a brief introduction.

This repository presents a self-synchronizing, low-power, low-complexity body-coupled communication (BCC)
transceiver using the edge coded signaling (ECS) / Pulsed-Index Communication (PIC) techniques. The unique 
features of these techniques are used to simplify the BCC transceiver hardware and reduce its power 
consumption by eliminating the need for circuitries dedicated to clock and data recovery (CDR) and
duty cycle correction. The self-synchronizing feature of the transceiver is achieved by exploiting the 
edge-coding property of ECS which consists of using pulse edges for encoding and detecting transmitted 
pulses rather than bit times or duty cycles. A working prototype of the proposed BCC transceiver
using off-the-shelf components is developed and used to test a full bi-directional BCC link by transmitting
arbitrary 16-bit data words through the human body over a range of 150cm with zero bit-error rate and 
sub-1nJ/bit energy efficiency.

The basic circuit to transmit and detect BCC signals is shown in Figure 1. The developed BCC transceiver 
circuit is shown in Figure 2. The demo setup is shown in Figure 3.
A judicious selection of the bit encoding protocol results in a significant simplification of the BCC 
transceiver. In particular, the selection of the edge-based encoding protocol as implemented in the ECS family 
results in a streamlined analog transceiver architecture that does not require the data conversion, data 
synchronization or duty cycle correction blocks.

![image](https://github.com/user-attachments/assets/86ce88d2-ea1c-45a0-add4-4ca1e31ecf58)

Figure 1: Base Architecture for Recovering BCC Signals


![image](https://github.com/user-attachments/assets/0e14e81c-b368-4fbb-a5e5-e166137244b6)

Figure 2: BCC Receiver Circuit

![image](https://github.com/user-attachments/assets/bb4cc890-44e3-423a-a085-2e25ee9d5b5a)

Figure 3: BCC Demo Setup: (a) Node1 (b) Node2 (c) Body Channel : (1) Processing Unit (2) BCC Transceiver

## For more details please consider the following publications.

1. BCC: S. Muzaffar & I. A. M. Elfadel, "**A Self-Synchronizing, Low-Power, Low-Complexity Transceiver for
   Body-Coupled Communication**". In 2019 41st Annual International Conference of the IEEE Engineering in Medicine and
   Biology Society, EMBC 2019, Berlin, Germany, (pp. 4036-4039). [![DOI:10.1109/EMBC.2019.8857045](https://zenodo.org/badge/DOI/10.1109/EMBC.2019.8857045.svg)](https://doi.org/10.1109/EMBC.2019.8857045) 
4. Shahzad Muzaffar and Ibrahim (Abe) M. Elfadel. 2020. **Dynamic Edge-coded Protocols for Low-power,
   Device-to-device Communication**. ACM Trans. Sen. Netw. 17, 1, Article 8 (February 2021), 24 pages.
   [![DOI:10.1145/3426181](https://zenodo.org/badge/DOI/10.1145/3426181.svg)](https://doi.org/10.1145/3426181)
1. ECS3 (PICplus): Shahzad Muzaffar and Ibrahim (Abe) M. Elfadel. 2020. **Dynamic Edge-coded Protocols for Low-power,
   Device-to-device Communication**. ACM Trans. Sen. Netw. 17, 1, Article 8 (February 2021), 24 pages.
   [![DOI:10.1145/3426181](https://zenodo.org/badge/DOI/10.1145/3426181.svg)](https://doi.org/10.1145/3426181)
2. ECS2 (PDC): S. Muzaffar and I. M. Elfadel, "**A pulsed decimal technique for single-channel, dynamic signaling for
   IoT applications**," 2017 IFIP/IEEE International Conference on Very Large Scale Integration (VLSI-SoC), Abu Dhabi,
   United Arab Emirates, 2017, pp. 1-6.[![DOI:10.1109/VLSI-SoC.2017.8203491](https://zenodo.org/badge/DOI/10.1109/VLSI-SoC.2017.8203491.svg)](https://ieeexplore.ieee.org/document/8203491)
4. ECS1 (PIC): S. Muzaffar, J. Yoo, A. Shabra and I. A. M. Elfadel, "**A pulsed-index technique for single-channel,
   low-power, dynamic signaling**," 2015 Design, Automation & Test in Europe Conference & Exhibition (DATE), Grenoble,
   France, 2015, pp. 1485-1490. [![DOI:10.7873/DATE.2015.1070](https://zenodo.org/badge/DOI/10.7873/DATE.2015.1070.svg)](https://ieeexplore.ieee.org/document/7092624)
5. DDR-ECS: S. Muzaffar and I. A. M. Elfadel, "**Double Data Rate Dynamic Edge-Coded Signaling for Low-Power IoT
   Communication**," 2019 IFIP/IEEE 27th International Conference on Very Large Scale Integration (VLSI-SoC),
   Cuzco, Peru, 2019, pp. 317-322. [![DOI:10.1109/VLSI-SoC.2019.8920318](https://zenodo.org/badge/DOI/10.1109/VLSI-SoC.2019.8920318.svg)](https://ieeexplore.ieee.org/document/8920318)
6. Secure ECS: S. Muzaffar, O. T. Waheed, Z. Aung and I. M. Elfadel, "**Lightweight, Single-Clock-Cycle, Multilayer
   Cipher for Single-Channel IoT Communication: Design and Implementation**," in IEEE Access, vol. 9, pp. 66723-66737,
   2021. [![DOI:10.1109/ACCESS.2021.3076468](https://zenodo.org/badge/DOI/10.1109/ACCESS.2021.3076468.svg)](https://ieeexplore.ieee.org/document/9419040)
