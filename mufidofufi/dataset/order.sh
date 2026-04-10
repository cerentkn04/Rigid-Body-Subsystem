#!/bin/bash

# simple bash script to order files starting with 0

count=0
for item in *; do
    echo "Found item: $item"
    echo "this is $count"
    mv "$item" "$count.png"
    let "count+=1"

    
done
