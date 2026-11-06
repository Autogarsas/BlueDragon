"""Read saved Panda startup entries for the current checkout; no USB access."""
from pathlib import Path
import json
import re
import subprocess

ROOT=Path(__file__).resolve().parents[2]


def main():
  release=json.loads((ROOT/'tools/ford_legacy_lka/release.json').read_text())
  if release.get('package_status') == 'native-build-pending':
    raise SystemExit('Fusion port H7 pairing is not packaged yet. Do not install or test this source-only revision.')
  firmware=json.loads((ROOT/release['h7_manifest']).read_text())
  version,prefix=firmware['build_version'],firmware['firmware_signature_prefix']
  commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
  print('Packaged firmware:',version,'signature prefix:',prefix)
  files=sorted(Path('/data/log').glob('swaglog.*'),key=lambda p:p.stat().st_mtime,reverse=True)
  for path in files:
    try:lines=path.read_text(errors='replace').splitlines()
    except OSError:continue
    for line in reversed(lines):
      if 'Panda ' not in line or 'connected, version:' not in line:continue
      try:record=json.loads(line)
      except ValueError:continue
      if record.get('ctx',{}).get('commit') != commit:continue
      message=record.get('msg$s',record.get('msg',''))
      if not isinstance(message,str):continue
      match=re.search(r'version: (\S+), signature ([0-9a-f]+), expected ([0-9a-f]+)',message)
      if not match:continue
      actual,loaded,expected=match.groups()
      if actual==version and loaded==expected==prefix:
        print('Matching Panda startup signature prefix found for this checkout.')
        print(message)
        return
  print('No matching post-flash startup entry for this checkout was found.')
  print('This does not verify the connected firmware. Send the startup output before testing.')


if __name__=='__main__':main()
