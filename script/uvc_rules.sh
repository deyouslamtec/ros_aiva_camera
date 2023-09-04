echo -e 'SUBSYSTEM=="usb", ATTRS{idProduct}=="0001", ATTRS{idVendor}=="8088", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"\nSUBSYSTEM=="tty", ATTRS{idProduct}=="0001", ATTRS{idVendor}=="8088", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"\nSUBSYSTEM=="usb", ATTRS{idProduct}=="0001", ATTRS{idVendor}=="8088", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"\nSUBSYSTEM=="tty", ATTRS{idProduct}=="0001", ATTRS{idVendor}=="8088", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"\n' >> ./99-aiva.rules
sudo mv ./99-aiva.rules /etc/udev/rules.d/
udevadm trigger
