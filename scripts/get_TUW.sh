#!/bin/bash

cd data
echo "Downloading TUW model database..."
output=$(wget -c -O TUW_models.tar.gz https://data.acin.tuwien.ac.at/index.php/s/63q18IDOvLYsvNO/download)
if [ $? -ne 0 ]; then
    echo "Error downloading file"
else
    echo "File has been downloaded"
    echo "Inflating file..."
    mkdir TUW
    cd TUW

    if ! tar -zxvf ../TUW_models.tar.gz &> /dev/null; then
        echo "Failure during inflating.."
    else
        echo "Successfully inflated file! Deleting tar file..."
        cd ..
        rm TUW_models.tar.gz
        echo "Done!"
    fi
fi


echo "Downloading TUW test set..."
output=$(wget -c -O TUW_test_set.tar.gz https://data.acin.tuwien.ac.at/index.php/s/BRimlBAPwoyOHmJ/download)
if [ $? -ne 0 ]; then
    echo "Error downloading file"
else
    echo "File has been downloaded"
    echo "Inflating file..."
    mkdir -p TUW/test_set
    cd TUW/test_set

    if ! tar -zxvf ../../TUW_test_set.tar.gz &> /dev/null; then
        echo "Failure during inflating.."
    else
        echo "Successfully inflated file! Deleting tar file..."
        cd ../..
        rm TUW_test_set.tar.gz
        echo "Done!"
    fi
fi



echo "Downloading ground-truth labels..."
output=$(wget -c -O TUW_annotations.tar.gz https://data.acin.tuwien.ac.at/index.php/s/GNzfAsEkd1kM5Uw/download)
if [ $? -ne 0 ]; then
    echo "Error downloading file"
else
    echo "File has been downloaded"
    echo "Inflating file..."
    mkdir -p TUW/annotations
    cd TUW/annotations

    if ! tar -zxvf ../../TUW_annotations.tar.gz &> /dev/null; then
        echo "Failure during inflating.."
    else
        echo "Successfully inflated file! Deleting tar file..."
        cd ../..
        rm TUW_annotations.tar.gz
        echo "Done!"
    fi
fi

cd ..
