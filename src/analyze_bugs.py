import os, re, sys

os.chdir(os.path.dirname(os.path.abspath(__file__)))

with open('optimus_prime_v13.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

issues = []

for i, l in enumerate(lines, 1):
    s = l.strip()
    
    # Check for unbalanced brackets in f-strings
    if 'entityOne' in l and 'entityTwo' in l:
        opened = l.count('[')
        closed = l.count(']')
        if opened != closed:
            issues.append((i, 'UNBALANCED BRACKETS in interference log line'))
    
    # Check for bare except clauses
    if s == 'except:':
        issues.append((i, 'BARE except (no Exception type)'))
    
    # Check for potential NameError: using vars before assignment
    if 'math.radians' in s and i < 800:
        pass  # math is imported at top
    
    # Check for potential unguarded attribute access
    if 'except Exception' in s or 'except:' in s:
        pass  # OK

print("=== BUG ANALYSIS REPORT for optimus_prime_g1_v8.py ===")
if issues:
    for ln, msg in issues:
        print(f"  [{ln}] {msg}")
        safe = lines[ln-1].rstrip()[:130].encode('ascii', 'replace').decode('ascii')
    print(f"    >>> {safe}")
else:
    print("No obvious issues found (basic checks passed).")

print()

# Count some statistics
funcs = [l for l in lines if l.strip().startswith('def ')]
classes = [l for l in lines if l.strip().startswith('class ')]
print(f"File: {len(lines)} lines, {len(funcs)} functions, {len(classes)} classes")

# Check all sim modules are in dispatch
dispatch_line = None
for i, l in enumerate(lines):
    if 'run_all_simulations' in l:
        dispatch_line = i
        break
if dispatch_line:
    print(f"Dispatch found at line {dispatch_line+1}")
