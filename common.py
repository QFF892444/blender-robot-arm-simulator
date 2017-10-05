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

scriptcache = {}
def load(name) :
    modulename = __package__+"."+name
    if modulename in scriptcache:
        if developing :
            importlib.reload(scriptcache[modulename])
    else:
        scriptcache[modulename] = importlib.import_module(modulename)
    return scriptcache[modulename]