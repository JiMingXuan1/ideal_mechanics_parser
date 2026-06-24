import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from io_handler.parser import parse_json
from core.engine import Engine
from io_handler.serializer import serialize_results


def main():
    if len(sys.argv) < 2:
        print("Usage: python main.py <input.json>")
        sys.exit(1)

    input_path = sys.argv[1]
    topology = parse_json(input_path)

    engine = Engine(topology)
    results = engine.run()

    os.makedirs("output", exist_ok=True)
    base = os.path.splitext(os.path.basename(input_path))[0]
    out_path = os.path.join("output", f"{base}_trajectory.json")
    serialize_results(results, out_path)

    print(f"Done. Results written to {out_path}")
    print(f"  Time steps: {len(results['t'])}")
    print(f"  Bodies: {len(results['node_order'])}")


if __name__ == "__main__":
    main()
