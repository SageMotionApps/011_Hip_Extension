# This script will run when the app is installed. Use it to install libraries or to do other work. In most cases, you don't need to use this though.

python3 -m venv ./venv
source ./venv/bin/activate
pip3 install -r requirements.txt
deactivate
mv ./venv/lib/python3.*/site-packages/ ./thiry_party
rm -r ./venv