#!/bin/sh

fl=$1

sed '1,$s/^\(.*\)|.*$/\1/g' $fl | sed '1,$s/,/\ /g' | sed '1,$s/;/\ /g' > obsC.txt
sed '1,$s/^.*|\(.*\)$/\1/g' $fl | sed '1,$s/,/\ /g' | sed '1,$s/;/\ /g' > toolC.txt
