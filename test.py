import onnx

p1 = "/home/kist/work/workspace/mjc-sangyun/pyfile/exported/policy.onnx"
p2 = "/home/kist/work/workspace/mjc-sangyun/pyfile/exported-513/policy.onnx"


def dims(v):
    return [
        d.dim_value if d.dim_value else d.dim_param
        for d in v.type.tensor_type.shape.dim
    ]


m1 = onnx.load(p1, load_external_data=False)
m2 = onnx.load(p2, load_external_data=False)

print("inputs\n")
for x in m1.graph.input:
    print(" ", x.name, dims(x))
for x in m2.graph.input:
    print("513 ", x.name, dims(x))
