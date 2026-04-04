# AWS Account Setup

**Page Owners:** [Mate Tirpak](https://github.com/matetirpak)

## AWS CLI Setup

1. Install the [AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html)

2. Request the Access Key and Secret Access Key from any of the infrastructure maintainers.

3. Configure the AWS CLI with the following command: 
    ```sh
    aws configure
    [access key]
    [secret access key]
    us-east-2
    json
    ```

4. You should now see your profile in ```~/.aws/config```. If you have multiple ones, run ```export AWS_PROFILE=``` with your desired profile name as the value and that profile will be used untill you close the bash session.
