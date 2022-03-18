#!/bin/bash

get_version() {
    version=2.0.4~$(git describe --always --tags --match "[0-9]*.[0-9]*.[0-9]*")
    echo ${version}
}

make_deb() {
	echo "Creating deb package..."
	build_dir=$(mktemp -d)
	mkdir ${build_dir}/DEBIAN
	mkdir -p ${build_dir}/usr/local/bin
	mkdir -p ${build_dir}/etc/udev/rules.d
	cp debian/control ${build_dir}/DEBIAN/
	cp debian/postinst ${build_dir}/DEBIAN/
	cp debian/postrm ${build_dir}/DEBIAN/
	cp movidius_usb_hotplug.sh ${build_dir}/usr/local/bin/
	cp 80-movidius-host.rules ${build_dir}/etc/udev/rules.d/
	chmod +x ${build_dir}/usr/local/bin/movidius_usb_hotplug.sh
	chmod 644 ${build_dir}/etc/udev/rules.d/80-movidius-host.rules

	get_version
	sed -i "s/VERSION/${version}/" ${build_dir}/DEBIAN/control
	cat ${build_dir}/DEBIAN/control
	echo depthai-ctrl-host_${version}_amd64.deb
	fakeroot dpkg-deb --build ${build_dir} ../depthai-ctrl-host_${version}_amd64.deb
	rm -rf ${build_dir}
	echo "Done"
}

version=$(get_version)
make_deb
