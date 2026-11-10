file(SHA256 "${UF2}" digest)
file(WRITE "${MANIFEST}" "{\n  \"board\": \"${BOARD}\",\n  \"protocol\": \"RCPv1\",\n  \"build_id\": \"${BUILD}\",\n  \"profile_id\": \"${PROFILE}\",\n  \"uf2\": \"rc_car_firmware.uf2\",\n  \"sha256\": \"${digest}\"\n}\n")
