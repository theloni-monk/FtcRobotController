import sys
f = open(sys.argv[2][0],'r').readlines()
print(len(f))
for i in range(len(f)):
    f[i] = f[i].split(",\"")
print(f)
j = [",\""+t[1] for t in f]
ss=""
for s in j:
    ss = ss+ s
ss = ss[2::] ##remove initial ,"
print(ss)
json = open(sys.argv[2][1],'w')
json.write(ss)
##its not perfect but its good enough fuck it
