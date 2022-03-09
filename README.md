# Operating Systems Lab - NTUA 2021-2022
## Members of OSLAB20:
  - Vasilis Vrettos - el18126
  - Alexandros Tsafos - el18211

### 1. Riddle
You find a Flash Drive in your pocket with a note: Forget the assignment, solve the Riddle.

The drive contains only one executable for Linux/x86-64. You take no risks, but you are curious. You run it in an isolated environment.

You quickly realise that it consists of a series of challenges. Each challenge has an outcome: SUCCESS or FAIL. If you get past a challenge you can take on the next one. (Or the next of the next...)

### 2. Lunix-TNG Character Device Driver
Implementation of a character device driver for a wireless mesh of sensors for the Operating System of Linux.

### 3. Cryptodev-VirtIO Encrypted Chat Service
Virtual Hardware development in the virtualization environment of QEMU/KVM.

This service is made up by 3 parts:
  1. A basic non-encrypted chat using TCP over IP sockets.
  2. Encryption support for the above-mentioned chat using the Linux Cryptodev device driver.
  3. Implementation of a VirtIO crypto device for the QEMU/KVM virtualization environment and the needed character device driver for the guest Linux Kernel.
