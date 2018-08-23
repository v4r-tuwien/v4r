#!/bin/bash

cd data
echo "Downloading test database from 3dNet..."
output=$(wget -c -O Cat10_TestDatabase.zip https://data.acin.tuwien.ac.at/index.php/s/6XkgKCAmpUyznnP/download)
if [ $? -ne 0 ]; then
    echo "Error downloading file"
else
    echo "File has been downloaded"
    echo "Unzipping..."
    mkdir -p 3dNet/Cat10_TestDatabase
    cd 3dNet/Cat10_TestDatabase

    unzip ../../Cat10_TestDatabase.zip
    if [ $? -ne 0 ]; then
        echo "Failure during unzipping.."
        echo "Still trying to use data..."
    else
        echo "Successfully unzipped file! Cleaning up..."
    fi
        
    #remove broken files
    echo "Remove files known to be broken"
    cd pcd_binary
    mv tt_tlt_ppr61.pcd tt_tlt_ppr61.pcd_broken
    mv tt_hmrr20.pcd tt_hmrr20.pcd_broken
    mv tt_hmrr89.pcd tt_hmrr89.pcd_broken
    mv tt_hmrr107.pcd tt_hmrr107.pcd_broken
    mv tt_hmrr116.pcd tt_hmrr116.pcd_broken
    mv tt_hmrr132.pcd tt_hmrr132.pcd_broken
    mv tt_hmrr141.pcd tt_hmrr141.pcd_broken
    cd ..
    
    cd ../..
    rm Cat10_TestDatabase.zip
    echo "Done!"
fi

cd ..

