#!/usr/bin/env python3
"""Download immutable vendor sources; never initializes submodules in the parent repo."""
import concurrent.futures
import hashlib
import io
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def fetch(entry):
    dest = ROOT / 'vendor' / entry['name']
    marker = dest / '.source-commit'
    if marker.exists() and marker.read_text().strip() == entry['commit']:
        return
    if entry.get('sparse_paths'):
        dest.mkdir(parents=True, exist_ok=True)
        if not (dest / '.git').exists():
            subprocess.run(['git', 'init', '-q', str(dest)], check=True)
            subprocess.run(['git', '-C', str(dest), 'remote', 'add', 'origin',
                            f"https://github.com/{entry['repo']}.git"], check=True)
        subprocess.run(['git', '-C', str(dest), 'sparse-checkout', 'init', '--cone'], check=True)
        subprocess.run(['git', '-C', str(dest), 'sparse-checkout', 'set', *entry['sparse_paths']], check=True)
        subprocess.run(['git', '-C', str(dest), '-c', 'remote.origin.promisor=true',
                        '-c', 'remote.origin.partialclonefilter=blob:none',
                        'fetch', '-q', '--depth', '1', '--filter=blob:none', 'origin', entry['commit']], check=True)
        subprocess.run(['git', '-C', str(dest), 'checkout', '-q', '--detach', entry['commit']], check=True)
        marker.write_text(entry['commit'] + '\n')
        return
    url = f"https://codeload.github.com/{entry['repo']}/tar.gz/{entry['commit']}"
    print(f"Fetching {entry['repo']} @ {entry['commit']}", flush=True)
    with urllib.request.urlopen(url, timeout=180) as response:
        data = response.read()
    if entry.get('sha256') and hashlib.sha256(data).hexdigest() != entry['sha256']:
        raise RuntimeError(f"Archive checksum mismatch: {entry['name']}")
    staging = dest.with_name(dest.name + '.tmp')
    shutil.rmtree(staging, ignore_errors=True)
    staging.mkdir(parents=True)
    with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as archive:
        prefix = archive.getmembers()[0].name.split('/')[0] + '/'
        for member in archive.getmembers():
            if member.name.startswith(prefix) and member.name != prefix:
                member.name = member.name[len(prefix):]
                archive.extract(member, staging, filter='data')
    shutil.rmtree(dest, ignore_errors=True)
    staging.rename(dest)
    marker.write_text(entry['commit'] + '\n')
    (dest / '.archive-sha256').write_text(hashlib.sha256(data).hexdigest() + '\n')


if __name__ == '__main__':
    entries = json.loads((ROOT / 'dependencies.json').read_text())['sources']
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(fetch, entries))
