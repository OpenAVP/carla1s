import logging

class ProjectFormatter(logging.Formatter):
    def format(self, record):
        # 增加 shortname 字段, 保留 name 字段最后一个 '.' 之后的内容
        if '.' in record.name:
            record.shortname = record.name.split('.')[-1]
        else:
            record.shortname = record.name

        # 继续调用父类的 format 方法
        return super().format(record)
