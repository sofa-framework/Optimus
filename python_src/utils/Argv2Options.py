import  pprint

def argv2options(argv, options): # loads values from command line arguments into the options dictionary. syntax: runSofa file.py --argv file.yml key [key...] value key [key...] value ...
                                 # the different values are concatenated and loaded into options[io][suffix]
    i = 0
    suffix = options['io']['suffix']
    suffixSDA = options['io']['sdaSuffix']
    print['argv', argv]
    while i < len(argv):        
        if argv[i] in options:
            key0 = argv[i]
            if argv[i+1] in options[key0]:
                key1 = argv[i+1]
                if argv[i+2] in options[key0][key1]:                    
                    key2 = argv[i+2]

                    existingType = type(options[key0][key1][key2])
                    if existingType ==  list:
                        index = int(argv[i+3])
                        existingType = type(options[key0][key1][key2][index])
                        value = argv[i+4]
                        options[key0][key1][key2][index] = existingType(value)
                        i = i + 1
                    else:
                        value = argv[i+3]                        
                        options[key0][key1][key2] = existingType(value)
                    if key0 == 'filter':
                        suffixSDA = suffixSDA + '_' + value
                    else:
                        suffix = suffix + '_' + value
                    i = i + 4
                else:
                    existingType = type(options[key0][key1])
                    if existingType ==  list:
                        index = int(argv[i+2])
                        existingType = type(options[key0][key1][index])
                        value = argv[i+3]
                        options[key0][key1][index] = existingType(value)
                        i = i + 1 
                    else:
                        value = argv[i+2]                        
                        options[key0][key1] = existingType(value)                                           
                    if key0 == 'filter':
                        suffixSDA = suffixSDA + '_' + value
                    else:
                        suffix = suffix + '_' + value
                    i = i + 3
            else:
                existingType = type(options[key0])
                if existingType ==  list:
                    index = int(argv[i+1])
                    existingType = type(options[key0][index])
                    value = argv[i+2]
                    options[key0][index] = existingType(value)
                    i = i + 1 
                else:
                    value = argv[i+1]                        
                    options[key0] = existingType(value)                                           
                if key0 == 'filter':
                    suffixSDA = suffixSDA + '_' + value
                else:
                    suffix = suffix + '_' + value
                i = i + 2
        else:
            print ('key needed i',i)
            return         

    options['io']['suffix'] = suffix
    options['io']['sdaSuffix'] = suffixSDA
    print('suffix', suffix)
    return options
