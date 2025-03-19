#!/bin/bash

set -e

PATH_PREFIX=${1:-.}

PACKAGE_VERSION_REGEX="^.*/damc-(.*)-damc.(tar\\.gz|zip)$"
for package in "$PATH_PREFIX"/damc-*-damc.tar.gz "$PATH_PREFIX"/damc-*-damc.zip; do
	if [[ "$package" =~ $PACKAGE_VERSION_REGEX ]] && [ -f "$package" ]; then
		PACKAGE_VERSION=${BASH_REMATCH[1]}

		echo "Repackaging $package with detected version $PACKAGE_VERSION"

		# Check if $package ends with tar.gz
		if [ "${package#*".tar.gz"}" != "$package" ]; then
			REPACKAGED_PACKAGE="$PATH_PREFIX/damc-gui-$PACKAGE_VERSION.tar.gz"
			gunzip -d < "$package" \
				| tar \
				--wildcards \
				--delete 'README*' \
				--delete 'damc_config.json' \
				--delete 'damc_server*' \
				--delete 'lib/libportaudio*' \
				--delete 'lib/libuv*' \
				--delete 'systemd' \
				| gzip > "$REPACKAGED_PACKAGE"
			comm -23 <(tar -tzf "$package" | sort) <(tar -tzf "$REPACKAGED_PACKAGE" | sort) | sed 's/^/deleting: /'
			tar -tzf "$REPACKAGED_PACKAGE" | sort | sed 's/^/remaining: /'
		else
			REPACKAGED_PACKAGE="$PATH_PREFIX/damc-gui-$PACKAGE_VERSION.zip"
			cp "$package" "$REPACKAGED_PACKAGE"
			zip --delete "$REPACKAGED_PACKAGE" \
				'README*' \
				'damc_config.json' \
				'damc_server*' \
				'libportaudio*' \
				'libuv*'
			zipinfo -1 "$REPACKAGED_PACKAGE" | sort | sed 's/^/remaining: /'
		fi
	else
		echo "Non supported $package"
	fi
done
