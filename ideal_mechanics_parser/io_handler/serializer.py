import json


def serialize_results(results, filepath):
    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
