package frc.robot.util.tuning;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public record Source(List<String> files, int line, String call, int argument, String field) {
    public static final Source NONE = new Source(List.of(), -1, null, -1, null);

    private static final String kRoot = "src/main/java/";

    public static Source caller(Class<?> api, String method, String call, int argument) {
        List<StackWalker.StackFrame> frames = StackWalker.getInstance().walk(stream -> stream.limit(16).toList());
        for (int i = 0; i < frames.size(); i++) {
            if (!isApi(frames.get(i), api, method)) {
                continue;
            }
            int next = i + 1;
            while (next < frames.size() && isApi(frames.get(next), api, method)) {
                next++;
            }
            if (next >= frames.size()) {
                return NONE;
            }
            StackWalker.StackFrame frame = frames.get(next);
            return new Source(List.of(file(frame)), frame.getLineNumber(), call, argument, null);
        }
        return NONE;
    }

    public static Source field(Object owner, String field) {
        return field(owner, field, null, -1);
    }

    public static Source field(Object owner, String field, String call, int argument) {
        List<String> files = new ArrayList<>();
        for (Class<?> type = owner.getClass(); type != null && type != Object.class; type = type.getSuperclass()) {
            files.add(file(type));
            if (declares(type, field)) {
                break;
            }
        }
        return new Source(List.copyOf(files), -1, call, argument, field);
    }

    public static double read(Object owner, String field) {
        try {
            Field found = find(owner.getClass(), field);
            found.setAccessible(true);
            return ((Number) found.get(owner)).doubleValue();
        } catch (ReflectiveOperationException | ClassCastException e) {
            throw new IllegalArgumentException("Can't tune " + owner.getClass().getSimpleName() + "." + field, e);
        }
    }

    public static String file(Class<?> type) {
        while (type.getEnclosingClass() != null) {
            type = type.getEnclosingClass();
        }
        return kRoot + type.getName().replace('.', '/') + ".java";
    }

    public Source withArgument(int argument) {
        return new Source(files, line, call, argument, field);
    }

    public boolean isKnown() {
        return !files.isEmpty();
    }

    public String describe() {
        if (files.isEmpty()) {
            return "";
        }
        String name = files.get(0).substring(files.get(0).lastIndexOf('/') + 1);
        if (field != null) {
            return name.replace(".java", "") + "." + field;
        }
        return name + ":" + line;
    }

    public Map<String, Object> toJson() {
        Map<String, Object> json = new LinkedHashMap<>();
        json.put("files", files);
        json.put("line", line);
        json.put("call", call);
        json.put("argument", argument);
        json.put("field", field);
        return json;
    }

    private static boolean isApi(StackWalker.StackFrame frame, Class<?> api, String method) {
        return frame.getClassName().equals(api.getName()) && frame.getMethodName().equals(method);
    }

    private static String file(StackWalker.StackFrame frame) {
        String name = frame.getClassName();
        int dot = name.lastIndexOf('.');
        String packagePath = dot < 0 ? "" : name.substring(0, dot).replace('.', '/') + "/";
        String fileName = frame.getFileName() != null
                ? frame.getFileName()
                : name.substring(dot + 1).replaceAll("\\$.*", "") + ".java";
        return kRoot + packagePath + fileName;
    }

    private static boolean declares(Class<?> type, String field) {
        try {
            type.getDeclaredField(field);
            return true;
        } catch (NoSuchFieldException e) {
            return false;
        }
    }

    private static Field find(Class<?> type, String field) throws NoSuchFieldException {
        for (Class<?> current = type; current != null; current = current.getSuperclass()) {
            try {
                return current.getDeclaredField(field);
            } catch (NoSuchFieldException e) {
                continue;
            }
        }
        throw new NoSuchFieldException(field);
    }
}
