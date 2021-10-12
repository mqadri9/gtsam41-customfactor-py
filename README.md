# ICS+GTSAM V4.1

These changes are based on the following example project: https://github.com/borglab/GTSAM-project-python

## ANACONDA ENVIRONMENT 
  Create a virtual python environment using [Anaconda](https://www.anaconda.com/products/individual):
  ```
  conda create -n ics python=3.7
  conda activate ics
  ```

## PYTHON PACKAGES
  ```
  pip install pyparsing
  pip install pybind 
  pip install matplotlib
  ```
  (pip version 9.0.3 might be required for pybind: `pip install pip==9.0.3`)

## GTSAM INSTALLATION 
  From the root directory:
  ```
    cd gtsam 
    mkdir build 
    cd build
    cmake -DGTSAM_BUILD_PYTHON=ON -DGTSAM_PYTHON_VERSION=3.7 ..
    make -j4 
    sudo make install 
  ```
  Note that `sudo make install` installs gtsam globally. If different GTSAM version (say v1 and v2) are to be used for different projects and assuming v1 is currently installed globally then running `sudo make install` from inside the build directory of GTSAM v2 overwrites the systems files with the specific GTSAM version files (i.e v2). (if `make` was already ran for both v2 and v1 then no need to run make again and switching between version is one with a sudo make install) 

## EXPOSING CHANGES TO GTSAM CORE IN THE PYTHON MODULE (OPTIONAL)
The following are necessary step to expose custom `c++` function to python.

If we want to add the custom function to the `NonLinearFactor` class:
```
virtual void printfromnonlinearfactor() const {
    std::cout << "print from non linear factor " << std::endl;
  };

```
Method should be added to `gtsampy.h` , `gtsam.h` and `gtsam/gtsam.i`  (paths relative to gtsam root directory)

Then from inside the gtsam/build directory, make and install gtsam again. 

Lastly:

`cd gtsam/build/python` and run `python setup.py install --force` and this will install the gtsam python package to the anaconda environment. 

Can verify by running `pip list | grep gtsam` and you should see: `gtsam (4.1.0)`


## GTSAM CUSTOM C++ PYTHON WRAPPER INSTALLATION 
  From the root directory:
  ```
    cd wrap 
    mkdir build 
    cd build
    cmake ..
    make -j4 
    sudo make install 
  ```

## INSTALLING ICS PACKAGES

 From the root directory:
  ``` 
    1- mkdir build
    2- cd build
    3- cmake .. 
    4- make -j4 
    5- sudo make install 
    6- make python-install 
  ```

Note that steps 4,5 and 6 above need to be ran again each time modifications to the ICS c++ files is done.  

(IF OPTIONAL STEP ABOVE) Note when running `make python-install`, you should see the following few line in the output :

```
Searching for gtsam==4.1.0
Best match: gtsam 4.1.0
Processing gtsam-4.1.0-py3.7.egg

```

(if no packages are found make python-install will install a pre-build python package from https://pypi.org/simple/gtsam/)

## Testing 
 From the root directory:
  ```
    cd python 
    python run_unary_constraints_1D.py
  ```

## DOCUMENTATION

For more detailed information, please refer to the [tutorial](TUTORIAL.md).


## LIST OF CHANGES 
1- Added the interface file `ics.i`

2- All code related to `QuadraticUnaryFactor1D` and `QuadraticBinaryFactor1D` are now included inside `cpp/ics2.h`

3- Added necessary includes inside `cpp/ics2.h`

4- Remove ``ord()`` call when creating a new factor. Change from:
```
  factor = ics.QuadraticUnaryFactor1D(gtsam.symbol(ord('x'), 0), np.array([0.]), 0, pose_noise)
```
to 
```
  factor = ics.QuadraticUnaryFactor1D(gtsam.symbol('x', 0), np.array([0.]), 0, pose_noise)
```

4- Changes to the CMakeLists.txt (based on the gtsam-project template)

