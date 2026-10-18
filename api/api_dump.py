#!/usr/bin/env python3
"""Flatten Doxygen XML into a Box2D public-API JSON file.

Reads the XML produced by Doxygen (configured in docs/CMakeLists.txt with
DOXYGEN_GENERATE_XML=YES) and writes a clean, stable schema to
api/box2d_api.json, or api/box2d_api_double.json for the double precision pass,
so binding generators in any language can consume it
without parsing C headers themselves.

Doxygen must run with B2_INLINE predefined as "static inline" so header-only
functions carry inline="yes". A double precision dump comes from a second
Doxygen pass with BOX2D_DOUBLE_PRECISION predefined. Each dump is complete for
its precision, bind against the one that matches the library build.

Schema 1, top level:
- schema: integer, bumped on any change to the shape below
- version: Box2D version, from the B2_VERSION macros unless given
- double_precision: true when the XML came from a BOX2D_DOUBLE_PRECISION pass
- files: public header names
- macros: name, params, function_like, value, header, brief
- constants: name, type, value, storage ("static" or "extern"), header, brief
- typedefs: name, type, args, header, brief, plus function {return_type, params}
  for callback types
- enums: name, values [{name, value (integer or null), expr, brief}], header, brief
- structs: name, fields [{name, type, args, brief, plus function for function
  pointer fields}], header, brief
- functions: name, symbol, inline, return_type, params [{name, type}], header,
  brief, details

A function with inline true is defined in the header and has no library symbol,
bindings must reimplement it. symbol is the name to link against, which differs
from name when a header macro renames the function, as b2CreateWorld is in the
double precision build.

With --check-library, the dump is compared against the exported symbols of a
shared library built with the same precision, and the script fails on any
mismatch. The path is the library or a build directory holding exactly one.
Windows DLL exports are read directly, other platforms use nm. See README.md
for the release recipe.
"""
import argparse
import ast
import json
import os
import re
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SCHEMA_VERSION = 1

TYPE_NOISE = ("B2_API", "B2_INLINE", "BOX2D_EXPORT", "extern", "static")

# Implementation macros that are not part of the API a binding should mirror
SKIP_MACROS = {
    "B2_API",
    "B2_INLINE",
    "B2_FORCE_INLINE",
    "B2_ALIGN_AS",
    "B2_LITERAL",
    "B2_ZERO_INIT",
    "B2_ID_INLINE",
    "BOX2D_EXPORT",
    "B2_ENABLE_VALIDATION",
}

# Exported utilities that base.h keeps out of the documented API with @cond
HIDDEN_EXPORTS = {
    "b2GetMilliseconds",
    "b2GetMillisecondsAndReset",
    "b2GetTicks",
    "b2Hash",
    "b2InternalAssert",
    "b2Yield",
}


def text_of(el):
    if el is None:
        return ""
    return " ".join("".join(el.itertext()).split())


def clean_type(s):
    return " ".join(t for t in s.split() if t not in TYPE_NOISE)


def clean_initializer(s):
    s = s.strip()
    return s[1:].strip() if s.startswith("=") else s


def header_of(el):
    loc = el.find("location")
    if loc is None:
        return ""
    return os.path.basename(loc.get("file", ""))


def split_params(args):
    """Split a C parameter list such as "(int a, const b2Vec2 *v)" into name and type pairs."""
    s = args.strip()
    if not s.startswith("("):
        return None
    depth = 0
    end = -1
    for i, ch in enumerate(s):
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                end = i
                break
    inner = s[1:end].strip()
    if inner in ("", "void"):
        return []

    parts = []
    depth = 0
    current = ""
    for ch in inner:
        if ch in "([":
            depth += 1
        elif ch in ")]":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append(current)
            current = ""
        else:
            current += ch
    parts.append(current)

    params = []
    for part in parts:
        part = " ".join(part.split())
        m = re.match(r"^(.*?[\s\*&])([A-Za-z_]\w*)\s*((?:\[[^\]]*\])*)$", part)
        if m and m.group(1).strip():
            params.append({"name": m.group(2), "type": clean_type(m.group(1).strip() + m.group(3))})
        else:
            params.append({"name": "", "type": clean_type(part)})
    return params


def function_signature(return_type, args):
    params = split_params(args)
    if params is None:
        return None
    return {"return_type": return_type, "params": params}


def eval_int(expr, names):
    """Evaluate an integer constant expression from an enum initializer, None if it isn't one."""
    expr = re.sub(r"(?<=[0-9a-fA-FxX])[uUlL]+\b", "", expr)
    try:
        tree = ast.parse(expr, mode="eval")
    except SyntaxError:
        return None

    def ev(node):
        if isinstance(node, ast.Expression):
            return ev(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, int):
            return node.value
        if isinstance(node, ast.Name) and node.id in names:
            return names[node.id]
        if isinstance(node, ast.UnaryOp):
            v = ev(node.operand)
            if isinstance(node.op, ast.USub):
                return -v
            if isinstance(node.op, ast.UAdd):
                return v
            if isinstance(node.op, ast.Invert):
                return ~v
        if isinstance(node, ast.BinOp):
            a = ev(node.left)
            b = ev(node.right)
            ops = {
                ast.Add: lambda: a + b,
                ast.Sub: lambda: a - b,
                ast.Mult: lambda: a * b,
                ast.LShift: lambda: a << b,
                ast.RShift: lambda: a >> b,
                ast.BitOr: lambda: a | b,
                ast.BitAnd: lambda: a & b,
                ast.BitXor: lambda: a ^ b,
            }
            if type(node.op) in ops:
                return ops[type(node.op)]()
        raise ValueError

    try:
        return ev(tree)
    except (ValueError, TypeError, RecursionError):
        return None


def parse_function(md):
    name = md.findtext("name", "").strip()
    return {
        "name": name,
        "symbol": name,
        "inline": md.get("inline") == "yes",
        "return_type": clean_type(text_of(md.find("type"))),
        "params": [
            {
                "name": (p.findtext("declname") or "").strip(),
                "type": clean_type(text_of(p.find("type")) + text_of(p.find("array"))),
            }
            for p in md.findall("param")
            if clean_type(text_of(p.find("type"))) != "void" or p.findtext("declname")
        ],
        "header": header_of(md),
        "brief": text_of(md.find("briefdescription")),
        "details": text_of(md.find("detaileddescription")),
    }


def parse_typedef(md):
    item = {
        "name": md.findtext("name", "").strip(),
        "type": clean_type(text_of(md.find("type"))),
        "args": text_of(md.find("argsstring")),
        "header": header_of(md),
        "brief": text_of(md.find("briefdescription")),
    }
    sig = function_signature(item["type"], item["args"])
    if sig is not None:
        item["function"] = sig
    return item


def parse_enum(md):
    names = {}
    values = []
    next_value = 0
    for ev in md.findall("enumvalue"):
        name = ev.findtext("name", "").strip()
        expr = clean_initializer(text_of(ev.find("initializer")))
        value = eval_int(expr, names) if expr else next_value
        if value is not None:
            names[name] = value
            next_value = value + 1
        else:
            next_value = None
        values.append(
            {
                "name": name,
                "value": value,
                "expr": expr,
                "brief": text_of(ev.find("briefdescription")),
            }
        )
    return {
        "name": md.findtext("name", "").strip(),
        "values": values,
        "header": header_of(md),
        "brief": text_of(md.find("briefdescription")),
    }


def parse_macro(md):
    params = [(p.findtext("defname") or "").strip() for p in md.findall("param")]
    return {
        "name": md.findtext("name", "").strip(),
        "params": params,
        "function_like": md.find("param") is not None,
        "value": text_of(md.find("initializer")),
        "header": header_of(md),
        "brief": text_of(md.find("briefdescription")),
    }


def parse_constant(md):
    return {
        "name": md.findtext("name", "").strip(),
        "type": clean_type(text_of(md.find("type"))),
        "value": clean_initializer(text_of(md.find("initializer"))),
        "storage": "static" if md.get("static") == "yes" else "extern",
        "header": header_of(md),
        "brief": text_of(md.find("briefdescription")),
    }


def parse_struct(cd):
    fields = []
    for sd in cd.findall("sectiondef"):
        for md in sd.findall("memberdef"):
            if md.get("kind") == "variable":
                field = {
                    "name": md.findtext("name", "").strip(),
                    "type": clean_type(text_of(md.find("type"))),
                    "args": text_of(md.find("argsstring")),
                    "brief": text_of(md.find("briefdescription")),
                }
                if field["type"].endswith("(*)"):
                    sig = function_signature(field["type"][:-3].strip(), field["args"])
                    if sig is not None:
                        field["function"] = sig
                fields.append(field)
    return {
        "name": cd.findtext("compoundname", "").strip(),
        "fields": fields,
        "header": header_of(cd),
        "brief": text_of(cd.find("briefdescription")),
    }


LIBRARY_PATTERNS = ("box2d*.dll", "libbox2d*.so*", "libbox2d*.dylib")


def find_library(path):
    path = Path(path)
    if path.is_file():
        return path
    if not path.is_dir():
        sys.exit(f"error: library not found: {path}")
    found = sorted({p for pattern in LIBRARY_PATTERNS for p in path.rglob(pattern) if p.is_file() and not p.is_symlink()})
    if len(found) != 1:
        listing = "".join(f"\n  {p}" for p in found) or " none"
        sys.exit(f"error: need exactly one Box2D shared library under {path}, found:{listing}")
    return found[0]


def read_pe_exports(data):
    """Names in the export table of a PE image, from the Microsoft PE format specification."""
    pe = struct.unpack_from("<I", data, 0x3C)[0]
    if data[pe : pe + 4] != b"PE\0\0":
        sys.exit("error: not a PE image")
    section_count, optional_size = struct.unpack_from("<H12xH", data, pe + 6)
    optional = pe + 24
    magic = struct.unpack_from("<H", data, optional)[0]
    directories = optional + (112 if magic == 0x20B else 96)
    export_rva, export_size = struct.unpack_from("<II", data, directories)
    if export_size == 0:
        return set()

    sections = []
    table = optional + optional_size
    for i in range(section_count):
        size, address, raw_size, raw_offset = struct.unpack_from("<IIII", data, table + 40 * i + 8)
        sections.append((address, max(size, raw_size), raw_offset))

    def offset(rva):
        for address, size, raw_offset in sections:
            if address <= rva < address + size:
                return rva - address + raw_offset
        sys.exit(f"error: bad PE rva {rva:#x}")

    directory = offset(export_rva)
    name_count, names_rva = struct.unpack_from("<I4xI", data, directory + 24)
    names = set()
    for i in range(name_count):
        start = offset(struct.unpack_from("<I", data, offset(names_rva) + 4 * i)[0])
        names.add(data[start : data.index(b"\0", start)].decode("ascii"))
    return names


def read_exports(library):
    data = library.read_bytes()
    if data[:2] == b"MZ":
        return read_pe_exports(data)
    if library.suffix == ".dylib":
        lines = subprocess.run(["nm", "-gU", str(library)], capture_output=True, text=True, check=True).stdout
        return {line.split()[-1].lstrip("_") for line in lines.splitlines() if line.strip()}
    lines = subprocess.run(["nm", "-D", "--defined-only", str(library)], capture_output=True, text=True, check=True).stdout
    return {line.split()[-1] for line in lines.splitlines() if line.strip()}


def check_symbols(api, library):
    exported = read_exports(library)

    linked = {fn["symbol"] for fn in api["functions"] if not fn["inline"]}
    constants = {c["name"] for c in api["constants"] if c["storage"] == "extern"}

    missing = sorted((linked | constants) - exported)
    inline_exported = sorted(fn["symbol"] for fn in api["functions"] if fn["inline"] and fn["symbol"] in exported)
    undumped = sorted(
        s for s in exported if s.startswith("b2") and s not in linked and s not in constants and s not in HIDDEN_EXPORTS
    )

    ok = True
    for label, items in (
        ("in the dump but not exported", missing),
        ("marked inline but exported", inline_exported),
        ("exported but not in the dump", undumped),
    ):
        if items:
            ok = False
            print(f"error: {len(items)} symbols {label}:", file=sys.stderr)
            for s in items:
                print(f"  {s}", file=sys.stderr)
    if ok:
        print(f"symbols ok against {library.name} ({len(linked)} functions, {len(constants)} extern constants)")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xml", default="build/docs/xml", help="Doxygen XML output dir")
    ap.add_argument("--out", default="api/box2d_api.json", help="output JSON path")
    ap.add_argument("--version", default="", help="Box2D version string, defaults to the B2_VERSION macros")
    ap.add_argument("--check-library", metavar="PATH", help="shared library, or a build dir holding one, to check against")
    args = ap.parse_args()

    xml_dir = Path(args.xml)
    if not xml_dir.is_dir():
        sys.exit(f"error: xml dir not found: {xml_dir}")

    index = ET.parse(xml_dir / "index.xml").getroot()

    api = {
        "schema": SCHEMA_VERSION,
        "version": args.version,
        "double_precision": False,
        "files": [],
        "macros": [],
        "constants": [],
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
                    elif mkind == "variable":
                        api["constants"].append(parse_constant(md))
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
    for key in ("macros", "constants", "typedefs", "enums", "structs", "functions"):
        api[key] = sorted(dedup(api[key]), key=lambda x: x.get("name", ""))

    api["macros"] = [m for m in api["macros"] if m["name"] not in SKIP_MACROS]

    # A macro that maps a function name to another identifier renames its link symbol
    functions = {fn["name"]: fn for fn in api["functions"]}
    renames = set()
    for m in api["macros"]:
        if m["name"] in functions and not m["function_like"] and re.fullmatch(r"[A-Za-z_]\w*", m["value"]):
            functions[m["name"]]["symbol"] = m["value"]
            renames.add(m["name"])
    api["macros"] = [m for m in api["macros"] if m["name"] not in renames]

    if not api["version"]:
        macros = {m["name"]: m["value"] for m in api["macros"]}
        parts = [macros.get(f"B2_VERSION_{k}", "") for k in ("MAJOR", "MINOR", "REVISION")]
        if all(p.isdigit() for p in parts):
            api["version"] = ".".join(parts)

    # The large world position type is its own struct only in double precision
    api["double_precision"] = any(s["name"] == "b2Pos" for s in api["structs"])

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(api, f, indent=2, ensure_ascii=False)
        f.write("\n")
    print(
        f"wrote {out} "
        f"({'double' if api['double_precision'] else 'float'}, "
        f"{len(api['functions'])} functions, "
        f"{sum(fn['inline'] for fn in api['functions'])} inline, "
        f"{len(api['structs'])} structs, "
        f"{len(api['enums'])} enums, "
        f"{len(api['typedefs'])} typedefs, "
        f"{len(api['constants'])} constants, "
        f"{len(api['macros'])} macros)"
    )

    if args.check_library and not check_symbols(api, find_library(args.check_library)):
        sys.exit(1)


if __name__ == "__main__":
    main()
