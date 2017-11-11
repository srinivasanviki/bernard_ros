from check import identification
from AgeGender import agegender
import test


example_image = 'LeeHsienLoong2.jpeg'

result_list = []

identificationResult = identification(example_image)


age, gender = agegender(example_image)

result_list.append(identificationResult)
result_list.append(age)
result_list.append(gender)
print result_list