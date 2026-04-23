import yaml
with open("src/edu_robot/config/params_default.yaml") as f:
    yaml.safe_load(f)
print("YAML OK")
