#!/bin/bash

set -e

BOARDS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
VERSION=$1

if [ -z ${VERSION} ]; then
	echo "Version not provided!"
	exit
elif [[ ! ${VERSION} =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
	echo "Version ${VERSION} is in valid format (should be e.g. 1.0.0)"
	exit
else
	echo "Building version ${VERSION}"
fi

echo "Building in ${BOARDS_DIR}"

ZIP_FILE_NAME="credit_card_midi-${VERSION}.zip"
git mv ${BOARDS_DIR}/hardware/* "${BOARDS_DIR}/hardware/${VERSION}"
cd "${BOARDS_DIR}/hardware"; zip -r "${BOARDS_DIR}/../${ZIP_FILE_NAME}" "${VERSION}"; cd -
SHA=$(shasum -a 256 "${BOARDS_DIR}/../${ZIP_FILE_NAME}" | awk '{print $1}')
SIZE=$(stat -f "%z" "${BOARDS_DIR}/../${ZIP_FILE_NAME}")

echo "SHA: ${SHA}, zip size: ${SIZE}"

PACKAGE_FILE="${BOARDS_DIR}/package_credit_card_midi_index.json"

echo "Updating package file ${PACKAGE_FILE}"
sed -e "s/\([0-9]*\.[0-9]*\.[0-9]*\)/${VERSION}/g" -i "" "${PACKAGE_FILE}"
sed -e "s/SHA\-256\:\(.*\)\\\"/SHA\-256\:${SHA}\\\"/" -i "" "${PACKAGE_FILE}"
sed -e "s/\(\\\"size\\\"\: \)\\\"\([0-9]*\)\\\"/\1\\\"${SIZE}\\\"/" -i "" "${PACKAGE_FILE}"

git add "${PACKAGE_FILE}"
git commit -m "Release v${VERSION}"
git tag "v${VERSION}"
git push --tags
hub release create v${VERSION} -a "${BOARDS_DIR}/../${ZIP_FILE_NAME}" -m "Release v${VERSION}"
