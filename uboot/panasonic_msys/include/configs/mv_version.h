/*
 * ROL version 
 */
#define CONFIG_ROL
#define BOARD_LATE_INIT
#define CONFIG_XMODEM_FILE_VALIDATE
#define CONFIG_SILENT_CONSOLE

#ifndef CONFIG_PRIVATE_STRING
#define CONFIG_PRIVATE_STRING	"5.5.03"
#define CONFIG_EXTRA_ENV_SETTINGS \
"rol_image_name=" "image1.bin" "\0"\
"quiet=" "1" "\0"\
"netdev=" "eth2" "\0"\
"othbootargs=" "ubi.mtd=1" "\0"\
"mtdparts=" "mtdparts=armada-nand:4m(uboot),250m@4m(rootfs)" "\0"\
"mtdids=" "nand0=armada-nand" "\0"\
"rol_sd_image_name=" "sd_runtime.rom" "\0"\
"rol_sd_device=" "usb 0:1" "\0"\
"rol_sd_next=" "usb reset; setenv ethact asx0; ubi part rootfs 2048; setenv bootargs maxcpus=1 root=/dev/ram0 rw rootfstype=ramfs rdinit=/sbin/init ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:off $console $othbootargs $mtdparts loglevel=$loglevel earlyprintk=serial,uart0,115200 $mvNetConfig; rolimage_sd_next" "\0"\
"ramfsboot=" "usb reset; setenv ethact asx0; ubi part rootfs 2048; setenv bootargs maxcpus=1 root=/dev/ram0 rw rootfstype=ramfs rdinit=/sbin/init ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:off $console $othbootargs $mtdparts loglevel=$loglevel earlyprintk=serial,uart0,115200 $mvNetConfig; rolimage" "\0" \
"bootcmd=" "run rol_sd_next" "\0"\
"rolnfs=" "usb reset; setenv ethact asx0; ubi part rootfs 2048; setenv bootargs maxcpus=1 root=/dev/nfs rw nfsroot=$serverip:$rootpath ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:off $console $othbootargs $mtdparts loglevel=$loglevel earlyprintk=serial,uart0,9600 $mvNetConfig; rolimage" "\0"\
"nfsboot=" "usb reset; setenv ethact asx0; setenv bootargs maxcpus=1 root=/dev/nfs rw nfsroot=$serverip:$rootpath ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:off $console $othbootargs $mtdparts imageaddr=$imageaddr loglevel=$loglevel earlyprintk=serial,uart0,9600 $mvNetConfig; tftp $linux_loadaddr $image_name; bootm $linux_loadaddr" "\0"\
"rol_tftp=" "usb reset; setenv ethact asx0; ubi part rootfs 2048; ubi create rootfs 0xEA00000; setenv bootargs maxcpus=1 root=/dev/ram0 rw rootfstype=ramfs rdinit=/sbin/init ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:off $console $othbootargs $mtdparts loglevel=$loglevel earlyprintk=serial,uart0,9600 $mvNetConfig; tftp $imageaddr $rol_image_name; rolimage_tftp" "\0"\
"rol_next=" "usb reset; setenv ethact asx0; ubi part rootfs 2048; setenv bootargs maxcpus=1 root=/dev/ram0 rw rootfstype=ramfs rdinit=/sbin/init ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:off $console $othbootargs $mtdparts loglevel=$loglevel earlyprintk=serial,uart0,9600 $mvNetConfig; tftp $imageaddr $rol_image_name; rolimage_tftp" "\0" \
"mfgdiags=" "ubi part rootfs;ubifsmount rootfs;ubifsload $linux_loadaddr mfg.bin;bootm $linux_loadaddr"
#endif							
