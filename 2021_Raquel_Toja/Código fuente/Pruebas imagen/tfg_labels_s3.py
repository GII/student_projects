# Análisis de imagen almacenada en un bucket de Amazon S3 (Labels)
# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# PDX-License-Identifier: MIT-0 (For details, see https://github.com/awsdocs/amazon-rekognition-developer-guide/blob/master/LICENSE-SAMPLECODE.)

import boto3


def detect_labels(photo, bucket):

    client = boto3.client("rekognition", aws_access_key_id="XXX", aws_secret_access_key="XXX", region_name="eu-west-1")

    response = client.detect_labels(Image={"S3Object": {"Bucket": "raqueltojaq2", "Name": "photo2.jpg"}}, MaxLabels=10)

    print("Detected labels for " + photo)
    print()
    for label in response["Labels"]:
        print("Label: " + label["Name"])
        print("Confidence: " + str(label["Confidence"]))
        print("Instances:")
        for instance in label["Instances"]:
            print("  Bounding box")
            print("    Top: " + str(instance["BoundingBox"]["Top"]))
            print("    Left: " + str(instance["BoundingBox"]["Left"]))
            print("    Width: " + str(instance["BoundingBox"]["Width"]))
            print("    Height: " + str(instance["BoundingBox"]["Height"]))
            print("  Confidence: " + str(instance["Confidence"]))
            print()

        print("Parents:")
        for parent in label["Parents"]:
            print("   " + parent["Name"])
        print("----------")
        print()
    return len(response["Labels"])


def main():
    photo = ""
    bucket = ""
    label_count = detect_labels(photo, bucket)
    print("Labels detected: " + str(label_count))


if __name__ == "__main__":
    main()
