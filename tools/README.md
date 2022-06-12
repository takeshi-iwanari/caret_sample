# Local tools for CARET

- caret_visualizer
    - Visualize node diagram using architecture.yaml created by CARET

```
sudo apt install graphviz graphviz-dev
pip3 install -r requirements.txt

python3 caret_visualizer.py --architecture_yaml_file architecture.yaml --target_path all
```
