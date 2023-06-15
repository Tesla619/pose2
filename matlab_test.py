import matlab
#import matlab.engine

eng = matlab.engine.start_matlab()
print(eng)

data = [1, 2, 3, 4, 5]
matlab_data = matlab.double(data)
eng.workspace['data'] = matlab_data


#eng.quit()
