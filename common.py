import bpy, sys, importlib

developing = False

def console_get():
    for area in bpy.context.screen.areas:
        if area.type == 'CONSOLE':
            for space in area.spaces:
                if space.type == 'CONSOLE':
                    return area, space
    return None, None


def output(*args):
    area, space = console_get()
    if space is None:
        return

    context = bpy.context.copy()
    context.update(dict(
        space=space,
        area=area,
    ))

    text = ""

    for v in args:
        if type(v) is str:
            text += v + "  "
        else:
            text += str(v) + "  "

    for line in text.split("\n"):
        bpy.ops.console.scrollback_append(context, text=line, type='OUTPUT')

# 自动注册模块中的类
def auto_register(moduleName) :
    module = importlib.sys.modules[moduleName]
    for name in dir(module):
        sth = getattr(module,name)
        if "bl_idname" in dir(sth) and sth.__module__==moduleName :
            bpy.utils.register_class(sth)

# 自动注销模块中的类
def auto_unregister(moduleName) :
    module = importlib.sys.modules[moduleName]
    for name in dir(module):
        sth = getattr(module,name)
        if "bl_idname" in dir(sth) and sth.__module__==moduleName :
            bpy.utils.unregister_class(sth)

# 自动重新加载修改过的模块
scriptcache = {}
def load(name) :

    global scriptcache

    modulename = __package__+"."+name

    # 调用 importlib.find_loader() 前
    # 需要先调用 importlib.import_module()
    def getmtime():
        loader = importlib.find_loader(modulename)
        return loader.path_stats(loader.get_filename())["mtime"]

    if modulename in scriptcache:
        mtime = getmtime()
        if scriptcache[modulename]["mtime"]!=mtime:
            output("reload module", modulename, scriptcache[modulename]["m"])
            
            importlib.reload(scriptcache[modulename]["m"])
            scriptcache[modulename]["mtime"] = mtime
    else:
        scriptcache[modulename] = {
            "m": importlib.import_module(modulename),
            "mtime": getmtime()
        }
    
    return scriptcache[modulename]["m"]


# 是否为 坐标系前置
def isPreposing() :
    return bpy.context.scene.preposingAxesZ