"""Fetch a seeded COCO val2017 sample with original image/license provenance.

Only the validation caption index and selected JPEGs are downloaded. HTTP
Range reads avoid downloading the full annotation archive. No captions are
included in the generated manifest.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import io
import json
from pathlib import Path
import random
import zipfile

from PIL import Image
import requests


ARCHIVE = "https://s3.amazonaws.com/images.cocodataset.org/annotations/annotations_trainval2017.zip"
INDEX = "annotations/captions_val2017.json"
IMAGE_ROOT = "https://s3.amazonaws.com/images.cocodataset.org/val2017/"


class RemoteZip(io.RawIOBase):
    def __init__(self, url):
        self.url = url
        self.position = 0
        with requests.get(url, headers={"Range": "bytes=0-0"}, stream=True, timeout=30) as response:
            response.raise_for_status()
            if response.status_code != 206:
                raise ValueError("Archive server must support byte ranges")
            self.length = int(response.headers["Content-Range"].rsplit("/", 1)[1])

    def seekable(self):
        return True

    def tell(self):
        return self.position

    def seek(self, offset, whence=0):
        self.position = (0, self.position, self.length)[whence] + offset
        if self.position < 0:
            raise ValueError("negative archive offset")
        return self.position

    def read(self, size=-1):
        size = min(self.length - self.position, size if size >= 0 else self.length)
        if size <= 0:
            return b""
        end = self.position + size - 1
        with requests.get(self.url, headers={"Range": f"bytes={self.position}-{end}"},
                          stream=True, timeout=60) as response:
            response.raise_for_status()
            if response.status_code != 206:
                raise ValueError("Archive server ignored byte range")
            data = response.content
        if len(data) != size:
            raise ValueError(f"short archive read: {len(data)} != {size}")
        self.position += size
        return data


def fetch_sample(output, count=100, seed=20261106, exclude_manifests=()):
    output.mkdir(parents=True, exist_ok=True)
    metadata_path = output / "coco-validation-images.json"
    if not metadata_path.exists():
        with zipfile.ZipFile(RemoteZip(ARCHIVE)) as archive:
            raw = archive.read(INDEX)
        metadata = json.loads(raw)
        metadata = {"images": metadata["images"], "licenses": metadata["licenses"],
                    "annotation_archive": ARCHIVE, "annotation_member": INDEX,
                    "annotation_sha256": hashlib.sha256(raw).hexdigest()}
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
    metadata = json.loads(metadata_path.read_text())
    excluded = {image["coco_id"] for path in exclude_manifests
                for image in json.loads(path.read_text())["images"]}
    population = sorted((image for image in metadata["images"] if image["id"] not in excluded),
                        key=lambda item: item["id"])
    selected = random.Random(seed).sample(population, count)
    licenses = {entry["id"]: entry for entry in metadata["licenses"]}

    def fetch(entry):
        path = output / entry["file_name"]
        url = IMAGE_ROOT + entry["file_name"]
        if not path.exists():
            response = requests.get(url, timeout=60)
            response.raise_for_status()
            data = response.content
            with Image.open(io.BytesIO(data)) as image:
                image.verify()
            path.write_bytes(data)
        with Image.open(path) as image:
            dimensions = list(image.size)
        return {"coco_id": entry["id"], "file": path.name, "download_url": url,
                "coco_url": entry.get("coco_url"), "flickr_url": entry.get("flickr_url"),
                "license": licenses[entry["license"]], "dimensions": dimensions,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}

    with ThreadPoolExecutor(max_workers=4) as executor:
        images = list(executor.map(fetch, selected))
    manifest = {"dataset": "COCO val2017", "source": "https://cocodataset.org/#download",
                "seed": seed, "count": count, "selection": "seeded sample without replacement, sorted COCO ID population",
                "annotation_sha256": metadata["annotation_sha256"], "images": images}
    if excluded:
        manifest["excluded_coco_ids"] = sorted(excluded)
        manifest["selection"] += ", excluding previously used background IDs"
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"images": len(images), "output": str(output)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--seed", type=int, default=20261106)
    parser.add_argument("--exclude-manifest", type=Path, action="append", default=[])
    args = parser.parse_args()
    fetch_sample(args.output, args.count, args.seed, args.exclude_manifest)
