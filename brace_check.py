count=0
with open('Core/Src/main.c', encoding='utf-8', errors='ignore') as f:
    for num,line in enumerate(f,1):
        opens=line.count('{')
        closes=line.count('}')
        if opens or closes:
            count += opens - closes
            print(f"{num:4d}: +{opens} -{closes} => balance {count}")
print('final count',count)
