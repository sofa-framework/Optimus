def argv2options(argv, options): # loads values from command line arguments into the options dictionary. syntax: runSofa file.py --argv file.yml key [key...] value key [key...] value ...
                                 # the different values are concatenated and loaded into options[io][suffix]
    i = 0
    suffix = options['io']['suffix']
    suffixSDA = options['io']['sdaSuffix']
    while i < len(argv):        
        if argv[i] in options:
            key0 = argv[i]
            if argv[i+1] in options[key0]:
                key1 = argv[i+1]
                if argv[i+2] in options[key0][key1]:                    
                    key2 = argv[i+2]
                    existingType = type(options[key0][key1][key2])
                    options[key0][key1][key2] = existingType(argv[i+3])
                    if key0 == 'filter':
                        suffixSDA = suffixSDA + '_' + argv[i+3]
                    else:
                        suffix = suffix + '_' + argv[i+3]
                    i = i + 4
                else:
                    existingType = type(options[key0][key1])
                    options[key0][key1] = existingType(argv[i+2])
                    if key0 == 'filter':
                        suffixSDA = suffixSDA + '_' + argv[i+2]
                    else:
                        suffix = suffix + '_' + argv[i+2]
                    i = i + 3
            else:
                existingType = type(options[key0])
                options[key0] = existingType(argv[i+1])
                if key0 == 'filter':
                    suffixSDA = suffixSDA + '_' + argv[i+1]
                else:
                    suffix = suffix + '_' + argv[i+1]
                i = i + 2
        else:
            print ('key needed')
            return         

    options['io']['suffix'] = suffix
    options['io']['sdaSuffix'] = suffixSDA
    print('suffix', suffix)
    return options
