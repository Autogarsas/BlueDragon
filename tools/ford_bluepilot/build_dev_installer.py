"""Configure only the reviewed installer's padded branch parameter; never execute it."""
import argparse
import hashlib
from pathlib import Path


def configure(source):
  if hashlib.sha256(source).hexdigest() != '07c5e5743a878d54c655a1309c72a87db44fa94a47bc85308516baa6e0d666d1':
    raise ValueError('Expected the reviewed BlueDragon 1.0 installer')
  old, new = b'bluedragon-1.0?', b'bluedragon-dev?'
  offset = 957198
  if source.count(old) != 1 or source[offset:offset+74] != old + b' '*58 + b'\0':
    raise ValueError('Unexpected padded branch parameter')
  return source[:offset] + new + source[offset+len(old):]


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('source', type=Path)
  parser.add_argument('output', type=Path)
  args = parser.parse_args()
  data = configure(args.source.read_bytes())
  with args.output.open('xb') as f:
    f.write(data)
  args.output.chmod(0o755)
  print(hashlib.sha256(data).hexdigest())


if __name__ == '__main__':
  main()
