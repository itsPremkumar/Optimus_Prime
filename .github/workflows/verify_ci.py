import sys, subprocess, os
here = os.path.dirname(os.path.abspath(__file__))
engine = os.path.join(here, '..', '..', '_devops_loop', 'verify.py')
if os.path.exists(engine):
    rc = subprocess.run([sys.executable, engine, '--repo', '.', '--json', 'report.json']).returncode
    sys.exit(rc)
if os.path.exists('package.json'):
    sys.exit(subprocess.run('npm ci && npm test', shell=True).returncode)
sys.exit(subprocess.run([sys.executable, '-m','compileall','-q','.'], shell=True).returncode)
