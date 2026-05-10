#!/usr/bin/env python3
"""Flatten Doxygen XML into a single Box2D public-API JSON file.

Reads the XML produced by Doxygen (configured in docs/CMakeLists.txt with
DOXYGEN_GENERATE_XML=YES) and writes a clean, stable schema to
api/box2d_api.json so binding generators in any language can consume it
without parsing C headers themselves.
"""
import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def text_of(el):
    if el is None:
        return ""
    return " ".join("".join(el.itertext()).split())


_TYPE_NOISE = ("B2_API", "B2_INLINE")


def clean_type(s):
    return " ".join(t for t in s.split() if t not in _TYPE_NOISE)


def clean_initializer(s):
    s = s.strip()
    return s[1:].strip() if s.startswith("=") else s


def header_of(el):
    loc = el.find("location")
    if loc is None:
        return ""
    return os.path.basename(loc.get("file", ""))


def parse_function(md):
    return {
        "name": md.findtext("name", "").strip(),
        "return_type": clean_type(text_of(md.find("type"))),
        "params": [
            {
                "name": (p.findtext("declname") or "").strip(),
                "type": clean_type(text_of(p.find("type"))),
            }
            for p in md.findall("param")
        ],
        "header": header_of(md),
        "brief": text_of(md.find("briefdescription")),
        "details": text_of(md.find("detaileddescription")),
    }


def parse_typedef(md):
    return {
        "name": md.findtext("name", "").strip(),
        "type": clean_type(text_of(md.find("type"))),
        "args": text_of(md.find("argsstring")),
        "header": header_of(md),
        "brief": text_of(md.find("briefdescription")),
    }


def parse_enum(md):
    return {
        "name": md.findtext("name", "").strip(),
        "values": [
            {
                "name": ev.findtext("name", "").strip(),
                "value": clean_initializer(text_of(ev.find("initializer"))),
                "brief": text_of(ev.find("briefdescription")),
            }
            for ev in md.findall("enumvalue")
        ],
        "header": header_of(md),
        "brief": text_of(md.find("briefdescription")),
    }


def parse_macro(md):
    return {
        "name": md.findtext("name", "").strip(),
        "params": [(p.findtext("defname") or "").strip() for p in md.findall("param")],
        "value": text_of(md.find("initializer")),
        "header": header_of(md),
        "brief": text_of(md.find("briefdescription")),
    }


def parse_struct(cd):
    fields = []
    for sd in cd.findall("sectiondef"):
        for md in sd.findall("memberdef"):
            if md.get("kind") == "variable":
                fields.append(
                    {
                        "name": md.findtext("name", "").strip(),
                        "type": clean_type(text_of(md.find("type"))),
                        "args": text_of(md.find("argsstring")),
                        "brief": text_of(md.find("briefdescription")),
                    }
                )
    return {
        "name": cd.findtext("compoundname", "").strip(),
        "fields": fields,
        "header": header_of(cd),
        "brief": text_of(cd.find("briefdescription")),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--xml", default="build/docs/xml", help="Doxygen XML output dir")
    ap.add_argument("--out", default="api/box2d_api.json", help="output JSON path")
    ap.add_argument("--version", default="", help="Box2D version string")
    args = ap.parse_args()

    xml_dir = Path(args.xml)
    if not xml_dir.is_dir():
        sys.exit(f"error: xml dir not found: {xml_dir}")

    index = ET.parse(xml_dir / "index.xml").getroot()

    api = {
        "version": args.version,
        "files": [],
        "macros": [],
        "typedefs": [],
        "enums": [],
        "structs": [],
        "functions": [],
    }

    for compound in index.findall("compound"):
        kind = compound.get("kind")
        refid = compound.get("refid")
        compound_xml = ET.parse(xml_dir / f"{refid}.xml").getroot()
        cd = compound_xml.find("compounddef")
        if cd is None:
            continue

        if kind == "file":
            name = cd.findtext("compoundname", "").strip()
            if name.endswith(".h"):
                api["files"].append(name)
        if kind in ("file", "group"):
            for sd in cd.findall("sectiondef"):
                for md in sd.findall("memberdef"):
                    mkind = md.get("kind")
                    if mkind == "function":
                        api["functions"].append(parse_function(md))
                    elif mkind == "typedef":
                        api["typedefs"].append(parse_typedef(md))
                    elif mkind == "enum":
                        api["enums"].append(parse_enum(md))
                    elif mkind == "define":
                        api["macros"].append(parse_macro(md))
        elif kind in ("struct", "union"):
            api["structs"].append(parse_struct(cd))

    def dedup(items):
        seen = set()
        out = []
        for it in items:
            key = (it.get("name", ""), it.get("header", ""))
            if key in seen:
                continue
            seen.add(key)
            out.append(it)
        return out

    api["files"] = sorted(set(api["files"]))
    for key in ("macros", "typedefs", "enums", "structs", "functions"):
        api[key] = sorted(dedup(api[key]), key=lambda x: x.get("name", ""))

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(api, f, indent=2, ensure_ascii=False)
        f.write("\n")
    print(
        f"wrote {out} "
        f"({len(api['functions'])} functions, "
        f"{len(api['structs'])} structs, "
        f"{len(api['enums'])} enums, "
        f"{len(api['typedefs'])} typedefs, "
        f"{len(api['macros'])} macros)"
    )


if __name__ == "__main__":
    main()
