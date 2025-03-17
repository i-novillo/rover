import bluetooth  # Requires pybluez, install with: pip install git+https://github.com/pybluez/pybluez.git#egg=pybluez

nearby_devices = bluetooth.discover_devices(lookup_names=True)
print(f"Found {len(nearby_devices)} devices.")

for addr, name in nearby_devices:
    print(f"  {addr} - {name}")