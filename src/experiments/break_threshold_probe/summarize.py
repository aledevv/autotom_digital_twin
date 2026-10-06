"""Print a table from a results jsonl: python3 summarize.py results.jsonl"""
import json, sys
print("solver gpu   N support       Fb[N] hz   W*[N]   W*/Fb  Fb/N(pred)")
for line in open(sys.argv[1]):
    r = json.loads(line); w = r["threshold_weight_n"]
    print(f"{r['solver']}    {int(r['gpu'])} {r['iterations']:>3} {r['support']:<12} {r['break_force_n']:>5} {r['hz']:>3} "
          f"{'None' if w is None else format(w, '7.4f')} {'' if w is None else format(w / r['break_force_n'], '7.4f')} "
          f"{r['prediction_break_force_over_n']:.4f}")
