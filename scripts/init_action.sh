#!/bin/bash

# Find template dir
if [ ! -d "./src/actions/template" ]; then
	echo "Unable to find [template] package directory, reconfigure script or run from projects root."
	exit
fi

echo -n "Enter action name : "
read action

# Convert name to camelcase [https://stackoverflow.com/questions/34420091/spinal-case-to-camel-case]
camel_case=$(sed -r 's/(^|-)(\w)/\U\2/g' <<<"$action")

if [ -d "./src/actions/$action" ]; then
	echo "A performer named '$action' already exists, remove it or choose another name."
	exit
fi

cp -r "./src/actions/template" "./src/actions/$action"

# Replace names
find "./src/actions/$action" -name '*.*' -exec sed -i -e "s/template/$action/g" {} \;
find "./src/actions/$action" -name '*.*' -exec sed -i -e "s/Template/$camel_case/g" {} \;

# Rename folders
for file in `find "./src/actions/$action" -name '*.*'` ;
do
	mv $file ${file//template/$action} ;
done

catkin build $action