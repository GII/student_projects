# Detección de rostros en una imagen almacenada en un bucket de Amazon S3
# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# PDX-License-Identifier: MIT-0 (For details, see https://github.com/awsdocs/amazon-rekognition-developer-guide/blob/master/LICENSE-SAMPLECODE.)

import boto3
import json


def detect_faces(photo, bucket):

    client = boto3.client("rekognition", aws_access_key_id="XXX", aws_secret_access_key="XXX", region_name="eu-west-1")

    response = client.detect_faces(
        Image={"S3Object": {"Bucket": "raqueltojaq2", "Name": "photo3.jpg"}}, Attributes=["ALL"]
    )

    print("Detected faces for " + photo)
    for faceDetail in response["FaceDetails"]:
        print(
            "The detected face is between "
            + str(faceDetail["AgeRange"]["Low"])
            + " and "
            + str(faceDetail["AgeRange"]["High"])
            + " years old"
        )
        print("Here are the other attributes:")
        print(json.dumps(faceDetail, indent=4, sort_keys=True))
    return len(response["FaceDetails"])


def main():
    photo = "photo"
    bucket = "bucket"
    face_count = detect_faces(photo, bucket)
    print("Faces detected: " + str(face_count))


if __name__ == "__main__":
    main()
