"""Parse RX JSON lines from a log workbook into flat columns.

Usage:
  python3 parse_rx_to_columns.py swarm_log_YYYY-MM-DD.xlsx swarm_log_YYYY-MM-DD_parsed_rx.xlsx

Requirements:
  pip install pandas openpyxl
"""

import sys
import json
import pandas as pd

RX_SHEET_NAME = "RxRaw"

def flatten(d, parent_key="", sep="."):
    """Flatten nested mappings into dot-notation keys."""
    out = {}
    if isinstance(d, dict):
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else str(k)
            if isinstance(v, dict):
                out.update(flatten(v, new_key, sep=sep))
            elif isinstance(v, list):
                out[new_key] = json.dumps(v, ensure_ascii=False)
            else:
                out[new_key] = v
    else:
        out[parent_key or "value"] = d
    return out

def main(in_path: str, out_path: str):
    """Read the log workbook and write the parsed output workbook."""
    rx = pd.read_excel(in_path, sheet_name=RX_SHEET_NAME)
    rx.columns = [c.strip() for c in rx.columns]

    if "timestamp" not in rx.columns or "raw_line" not in rx.columns:
        raise ValueError(f'Expected columns ["timestamp","raw_line"] in sheet "{RX_SHEET_NAME}". Found: {list(rx.columns)}')

    records = []
    all_keys = set()
    errors = []

    for _, row in rx.iterrows():
        ts = row["timestamp"]
        raw_line = row["raw_line"]

        rec = {"timestamp": ts, "parse_ok": False, "raw_line": raw_line}

        if isinstance(raw_line, str) and raw_line.strip():
            try:
                obj = json.loads(raw_line.strip())
                flat = flatten(obj)
                all_keys.update(flat.keys())
                rec.update(flat)
                rec["parse_ok"] = True
            except Exception as e:
                rec["parse_error"] = str(e)
                errors.append({"timestamp": ts, "raw_line": raw_line, "error": str(e)})
        else:
            rec["parse_error"] = "raw_line not a non-empty string"
            errors.append({"timestamp": ts, "raw_line": raw_line, "error": rec["parse_error"]})

        records.append(rec)

    all_keys_sorted = sorted(all_keys)

    cols = ["timestamp", "parse_ok"] + all_keys_sorted + ["raw_line"]
    parsed_df = pd.DataFrame(records)

    # Ensure all columns exist
    for c in cols:
        if c not in parsed_df.columns:
            parsed_df[c] = pd.NA
    parsed_df = parsed_df[cols]

    fields_df = pd.DataFrame({"field": all_keys_sorted})
    errors_df = pd.DataFrame(errors)

    with pd.ExcelWriter(out_path, engine="openpyxl") as writer:
        parsed_df.to_excel(writer, sheet_name="ParsedRx", index=False)
        fields_df.to_excel(writer, sheet_name="Fields", index=False)
        rx.to_excel(writer, sheet_name="RxRaw_Source", index=False)
        if not errors_df.empty:
            errors_df.to_excel(writer, sheet_name="ParseErrors", index=False)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 parse_rx_to_columns.py <input_log.xlsx> <output_parsed.xlsx>")
        sys.exit(2)
    main(sys.argv[1], sys.argv[2])
