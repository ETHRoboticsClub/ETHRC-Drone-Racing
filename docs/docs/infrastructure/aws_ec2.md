# AWS EC2 Instances

**Page Owners:** [Mate Tirpak](https://github.com/matetirpak)

## Important

Our instances are comparatively cheap so you don't have to worry about costs when running them. However, please double check that your instance has stopped whenever you decide to quit. You can stop them either by an internal shutdown or through the AWS CLI.

## Inspect/Start/Stop Instances

The AWS CLI provides you with the following commands:

```sh
TEAM=rl  # Replace with your team: rl | mpc | perception

# List all instances
aws ec2 describe-instances \
  --filters "Name=tag:Team,Values=$TEAM" \
  --query "Reservations[].Instances[].[InstanceId, Tags[?Key=='Name'].Value | [0], State.Name, PublicIpAddress]" \
  --output table

# Start instances
aws ec2 start-instances --instance-ids i-xxxx

# Stop instances
aws ec2 stop-instances --instance-ids i-xxxx
```

## Connecting to an Instance

### Option 1 (Recommended): DCV Client (Desktop GUI)

Download the [DCV Client](https://www.amazondcv.com/)

Enter https://your_instance_public_ip:8443

Username: ubuntu

Password: [Please reach out to one of the maintainers]

---

### Option 2: Terminal

Run the following command after you replace the ssh key path as well the IP with your instance's public IP.
``` sh
ssh -i ~/.ssh/id_ed25519 ubuntu@127.0.0.1
```
Username: ubuntu

Password: [Please reach out to one of the maintainers]


## Instance Software and Compute

All instances are currently designed for IsaacSim + IsaacLab. Nevertheless, each instance is personal and you are free to modify yours in any way you wish, including the removal of IsaacLab and installation of other simulators such as Gazebo. If you would like some specific software to come preinstalled with your instance or you need stronger compute power, please reach out to the maintainers.
